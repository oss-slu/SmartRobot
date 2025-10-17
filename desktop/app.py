from flask import Flask, render_template, jsonify
from flask_socketio import SocketIO
import asyncio
import json
import threading
import time
import websockets
import struct
import traceback

# ---------------------------
# Flask / SocketIO setup
# ---------------------------
app = Flask(__name__)
app.config['SECRET_KEY'] = 'your-secret-key'
socketio = SocketIO(app, cors_allowed_origins="*")

robot_status = "OFF"

# ---------------------------
# Foxglove protocol
# ---------------------------
# Client->Server binary frame for a message:
#   [0]    : 0x01 (MESSAGE_DATA)
#   [1..4] : channelId (uint32, LE)
#   [5.. ] : payload bytes (JSON for our case)
MESSAGE_DATA_OPCODE = b"\x01"

# Server->Client binary frames (when you subscribe) include a timestamp.
# Header = opcode(1) + channelId(uint32 LE, 4) + timestamp(uint64 LE, 8) = 13 bytes
SERVER_FRAME_HEADER_LEN = 1 + 4 + 8

def build_json_payload_for_string(data_text: str) -> bytes:
    """
    For std_msgs/msg/String with encoding='json', the payload is simply:
      {"data":"<your text>"}
    encoded as UTF-8 bytes.
    """
    return json.dumps({"data": data_text}).encode("utf-8")

async def client_advertise_string_json(ws, channel_id: int, topic="/command"):
    """
    Advertise a client-publish channel for std_msgs/msg/String using JSON encoding.
    """
    msg = {
        "op": "advertise",
        "channels": [
            {
                "id": channel_id,
                "topic": topic,
                "encoding": "json",                 # <--- JSON insead of cdr
                "schemaName": "std_msgs/msg/String",
                "schemaEncoding": "ros2msg",        # optional but nice
                "schema": "string data\n",
            }
        ],
    }
    await ws.send(json.dumps(msg))

# ---------------------------
# Foxglove Bridge Client
# ---------------------------
class FoxgloveBridge:
    def __init__(self):
        self.websocket = None
        self.pi_ip = "10.178.43.127"   # <-- it changes!! check it!!
        self.port = 8765
        self.running = False
        self.connected = False
        self.subscribed_topics = {}
        self.command_channel_id = 4     #just random num at the moment
        # Subscribe to server topics here, will add more in the future
        self.topics_to_subscribe = ["/test_talker"]

    async def connect_to_pi_server(self):
        """Connect to Foxglove WebSocket server on Raspberry Pi."""
        try:
            socketio.emit("log", f"Attempting to connect to {self.pi_ip}:{self.port} …")
            self.websocket = await websockets.connect(
                f"ws://{self.pi_ip}:{self.port}",
                subprotocols=["foxglove.websocket.v1"],
            )
            if self.websocket is None:
                raise RuntimeError("WebSocket connection failed (None)")

            socketio.emit("log", "✓ WebSocket CONNECTED (foxglove.websocket.v1)")
            self.running = True
            self.connected = True

            # Start background publisher for /command (JSON)
            asyncio.create_task(self.start_command_sender_json())

            # Read incoming control/data messages
            await self.handle_messages()

        except Exception as e:
            socketio.emit("log", f"✗ WebSocket ERROR: {e}")
            traceback.print_exc()
            self.connected = False
        finally:
            self.running = False
            if self.websocket:
                try:
                    await self.websocket.close()
                except Exception:
                    pass
                self.websocket = None
            socketio.emit("log", "✗ WebSocket DISCONNECTED")

    async def handle_messages(self):
        """Handle incoming server messages (advertise, binary message frames, etc.)."""
        if not self.connected or self.websocket is None:
            socketio.emit("log", "⚠️ Cannot handle messages - not connected")
            return

        try:
            async for message in self.websocket:
                if not (self.running and self.connected and self.websocket is not None):
                    break

                try:
                    if isinstance(message, str):
                        # Text protocol frames (JSON control messages)
                        data = json.loads(message)
                        op = data.get("op")

                        if op == "advertise":
                            channels = data.get("channels", [])
                            socketio.emit("log", f"📋 Server advertised {len(channels)} channel(s)")
                            for ch in channels:
                                topic = ch.get("topic", "unknown")
                                channel_id = ch.get("id")
                                encoding = ch.get("encoding", "unknown")
                                socketio.emit("log", f" • {topic} (channel={channel_id}, encoding={encoding})")
                                if topic in self.topics_to_subscribe:
                                    await self.subscribe_to_channel(channel_id, topic)
                        else:
                            socketio.emit("log", f"→ Protocol message: {data}")

                    elif isinstance(message, (bytes, bytearray)):
                        # Binary data frames from server: 1+4+8 header + payload
                        b = bytes(message)
                        if len(b) < SERVER_FRAME_HEADER_LEN:
                            socketio.emit("log", f"⚠️ Short binary frame (len={len(b)})")
                            continue
                        if b[0:1] != MESSAGE_DATA_OPCODE:
                            socketio.emit("log", f"⚠️ Unknown opcode {b[0]:02x} in binary frame")
                            continue

                        channel_id = struct.unpack_from("<I", b, 1)[0]
                        timestamp_ns = struct.unpack_from("<Q", b, 1 + 4)[0]
                        payload = b[SERVER_FRAME_HEADER_LEN:]

                        # Best-effort: try to treat payload as JSON; if not, show a snippet
                        try:
                            txt = payload.decode("utf-8")
                            maybe_json = json.loads(txt)
                            socketio.emit("ros_message", {
                                "channel_id": channel_id,
                                "timestamp_ns": timestamp_ns,
                                "data": maybe_json,
                            })
                            socketio.emit("log", f"→ [{channel_id}] JSON @ {timestamp_ns} ns: {maybe_json}")
                        except Exception:
                            # Not JSON, just show first bytes
                            socketio.emit("ros_message", {
                                "channel_id": channel_id,
                                "timestamp_ns": timestamp_ns,
                                "data": "<non-JSON payload>",
                            })
                            head = payload[:32]
                            socketio.emit("log", f"→ [{channel_id}] Non-JSON payload ({len(payload)}B) @ {timestamp_ns} ns: {head.hex()} …")

                except json.JSONDecodeError as e:
                    socketio.emit("log", f"⚠️ JSON decode error: {e}")
                except Exception as e:
                    socketio.emit("log", f"⚠️ Message processing error: {e}")
                    traceback.print_exc()

        except Exception as e:
            socketio.emit("log", f"⚠️ Message handling error: {e}")
            traceback.print_exc()
            self.connected = False

    async def subscribe_to_channel(self, channel_id, topic):
        """Subscribe to a server-advertised channel (for receiving)."""
        try:
            subscription_id = len(self.subscribed_topics) + 1
            subscribe_msg = {
                "op": "subscribe",
                "subscriptions": [
                    {"id": subscription_id, "channelId": channel_id, "topic": topic}
                ],
            }
            await self.websocket.send(json.dumps(subscribe_msg))
            self.subscribed_topics[subscription_id] = topic
            socketio.emit("log", f"✓ Subscribed: {topic} (subId={subscription_id}, channelId={channel_id})")
        except Exception as e:
            socketio.emit("log", f"⚠️ Error subscribing to {topic}: {e}")

    # ---- publishing (/command) using JSON ----
    async def start_command_sender_json(self):
        """Advertise /command (JSON) and periodically send 'run'."""
        try:
            await client_advertise_string_json(self.websocket, self.command_channel_id, "/command")
            socketio.emit("log", f"✓ ClientAdvertised /command (JSON, channel {self.command_channel_id})")
            await asyncio.sleep(0.2)

            seq = 0
            while self.running and self.connected:
                payload = build_json_payload_for_string("run")
                frame = (
                    MESSAGE_DATA_OPCODE
                    + struct.pack("<I", self.command_channel_id)  # channelId (LE)
                    + payload                                      # JSON payload
                )
                await self.websocket.send(frame)
                socketio.emit("log", f"→ Sent /command (JSON): 'run' (seq={seq})")
                seq += 1
                await asyncio.sleep(3.0)

        except Exception as e:
            socketio.emit("log", f"✗ Error in command sender: {e}")
            traceback.print_exc()

# Global instance
foxglove_bridge = FoxgloveBridge()

# ---------------------------
# Flask routes
# ---------------------------
# @app.route("/")
# def index():
#     try:
#         return render_template("loading.html")
#     except Exception:
#         return "Loading..."

# @app.route("/home")
# def home():
#     try:
#         return render_template("index.html")  
#     except Exception:
#         return "Home page not found."

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/my_robot")
def my_robot():
    return render_template("my_robot.html")

@app.route("/status")
def status():
    return jsonify({"status": robot_status})

@app.route("/set_status/<new_status>")
def set_status(new_status):
    global robot_status
    robot_status = new_status
    return jsonify({"status": robot_status})

# ---------------------------
# Thread runner for the asyncio foxglove client
# ---------------------------
def start_foxglove_client():
    """Run the Foxglove client in a dedicated background thread with its own event loop."""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        loop.run_until_complete(foxglove_bridge.connect_to_pi_server())
    except Exception as e:
        socketio.emit("log", f"Error starting Foxglove client: {e}")
        traceback.print_exc()
    finally:
        loop.close()

# ---------------------------
# Main
# ---------------------------
if __name__ == "__main__":
    # Single client instance (avoid Flask reloader duplicates)
    client_thread = threading.Thread(target=start_foxglove_client, daemon=True)
    client_thread.start()

    socketio.run(app, host="0.0.0.0", port=8080, debug=False, use_reloader=False)
