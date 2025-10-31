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
def parse_cdr_string(payload: bytes) -> str:
    """
    Parse CDR-encoded std_msgs/msg/String payload.
    CDR format: [encapsulation_header(4)] [string_length(4)] [string_data]
    """
    if len(payload) < 8:
        return "<invalid CDR payload>"
    
    # Skip encapsulation header (4 bytes)
    # Read string length (little-endian uint32)
    string_length = struct.unpack("<I", payload[4:8])[0]
    
    if len(payload) < 8 + string_length:
        return "<truncated CDR payload>"
    
    # Extract string data
    string_data = payload[8:8+string_length].decode("utf-8", errors="replace")
    return string_data

async def client_advertise_twist_json(ws, channel_id: int, topic="/diff_drive_base/cmd_vel"):
    """
    Advertise a client-publish channel for geometry_msgs/msg/TwistStamped using JSON encoding.
    """
    msg = {
        "op": "advertise",
        "channels": [
            {
                "id": channel_id,
                "topic": topic,
                "encoding": "json",
                "schemaName": "geometry_msgs/msg/TwistStamped",
                "schemaEncoding": "ros2msg",
                "schema": (
                    "std_msgs/msg/Header header\n"
                    "  builtin_interfaces/msg/Time stamp\n"
                    "    int32 sec\n"
                    "    uint32 nanosec\n"
                    "  string frame_id\n"
                    "geometry_msgs/msg/Twist twist\n"
                    "  geometry_msgs/msg/Vector3 linear\n"
                    "    float64 x\n"
                    "    float64 y\n"
                    "    float64 z\n"
                    "  geometry_msgs/msg/Vector3 angular\n"
                    "    float64 x\n"
                    "    float64 y\n"
                    "    float64 z\n"
                ),
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
        self.pi_ip = "10.178.43.127"   # <-- it changes!! check it!
        self.port = 8765
        self.running = False
        self.connected = False
        self.subscribed_topics = {}
        self.command_channel_id = 4     #just random num at the moment
        self.connection_loop = None
        # Subscribe to server topics here, will add more in the future
        self.topics_to_subscribe = ["/test_talker", "/diff_drive_base/odom"]
        self.channel_info = {}  # channel_id → {topic, encoding, schemaName}

    async def connect_to_pi_server(self):
        """Connect to Foxglove WebSocket server on Raspberry Pi."""
        try:
            self.connection_loop = asyncio.get_event_loop()  
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
                                schema_name = ch.get("schemaName", "")
                                self.channel_info[channel_id] = {"topic": topic, "encoding": encoding, "schemaName": schema_name}
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
                            info = self.channel_info.get(channel_id, {})
                            schema = info.get("schemaName")
                            encoding = info.get("encoding")

                            # Skip ROS log frames from /rosout to avoid noise
                            if schema == "rcl_interfaces/msg/Log":
                                continue

                            if schema == "std_msgs/msg/String" and encoding == "cdr":
                                try:
                                    cdr_string = parse_cdr_string(payload)
                                    socketio.emit("ros_message", {
                                        "channel_id": channel_id,
                                        "timestamp_ns": timestamp_ns,
                                        "data": {"data": cdr_string},
                                    })
                                    socketio.emit("log", f"→ [{channel_id}] CDR @ {timestamp_ns} ns: {cdr_string}")
                                except Exception:
                                    socketio.emit("log", f"→ [{channel_id}] Unable to parse CDR String ({len(payload)}B)")
                            else:
                                head = payload[:32]
                                socketio.emit("ros_message", {
                                    "channel_id": channel_id,
                                    "timestamp_ns": timestamp_ns,
                                    "data": "<non-JSON payload>",
                                })
                                socketio.emit("log", f"→ [{channel_id}] {schema or 'unknown'} with {encoding or 'unknown'} encoding not decoded; first bytes: {head.hex()} …")

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
                    {"id": subscription_id, "channelId": channel_id, "topic": topic, "encoding": "json"}
                ],
            }
            await self.websocket.send(json.dumps(subscribe_msg))
            self.subscribed_topics[subscription_id] = topic
            socketio.emit("log", f"✓ Subscribed: {topic} (subId={subscription_id}, channelId={channel_id})")
        except Exception as e:
            socketio.emit("log", f"⚠️ Error subscribing to {topic}: {e}")

    # ---- publishing (/command) using JSON for Twist ----
    async def start_command_sender_json(self):
        """Advertise /command (JSON Twist)"""
        try:
            await client_advertise_twist_json(self.websocket, self.command_channel_id, "/diff_drive_base/cmd_vel")
            socketio.emit("log", f"✓ ClientAdvertised /command (JSON Twist, channel {self.command_channel_id})")
            await asyncio.sleep(0.2)
            
        except Exception as e:
            socketio.emit("log", f"✗ Error in command sender: {e}")
            traceback.print_exc()


    async def send_twist_command_once(self, linear_x=0.0, linear_y=0.0, linear_z=0.0, 
                                        angular_x=0.0, angular_y=0.0, angular_z=0.0,
                                        frame_id="base_link"):
        """Send a one-off JSON TwistStamped /command message"""
        if not (self.connected and self.websocket):
            socketio.emit("log", "⚠️ Not connected; cannot send command")
            return
        
        # Get current timestamp
        now = time.time()
        sec = int(now)
        nanosec = int((now - sec) * 1e9)
        
        twist_stamped_data = {
            "header": {
                "stamp": {
                    "sec": 0,
                    "nanosec": 0
                },
                "frame_id": frame_id
            },
            "twist": {
                "linear": {"x": linear_x, "y": linear_y, "z": linear_z},
                "angular": {"x": angular_x, "y": angular_y, "z": angular_z}
            }
        }
        
        payload = json.dumps(twist_stamped_data).encode("utf-8")
        frame = MESSAGE_DATA_OPCODE + struct.pack("<I", self.command_channel_id) + payload
        await self.websocket.send(frame)
        socketio.emit("log", f"→ Sent /diff_drive_base/cmd_vel (TwistStamped): linear=({linear_x},{linear_y},{linear_z}), angular=({angular_x},{angular_y},{angular_z})")
        

            
# Global instance
foxglove_bridge = FoxgloveBridge()

# ---------------------------
# Flask routes
# ---------------------------
@app.route("/")
def index():
    try:
        return render_template("loading.html")
    except Exception:
        return "Loading..."

@app.route("/home")
def home():
    try:
        return render_template("index.html")  
    except Exception:
        return "Home page not found."


@app.route("/my_robot")
def my_robot():
    return render_template("my_robot.html")

@socketio.on("connect_robot")
def handle_connect_robot(data):
    """Handle robot connection request from UI"""
    ip = data.get("ip", "10.178.43.127")
    socketio.emit("log", f"UI requested connection to {ip}")
    
    # Update the bridge IP and trigger connection
    foxglove_bridge.pi_ip = ip
    loop = asyncio.get_event_loop()
    loop.create_task(foxglove_bridge.connect_to_pi_server())

@socketio.on("disconnect_robot")
def handle_disconnect_robot():
    """Handle robot disconnection request from UI"""
    socketio.emit("log", "UI requested disconnection")
    foxglove_bridge.running = False
    foxglove_bridge.connected = False
    socketio.emit("robot_disconnected")

@socketio.on("check_robot_status")
def handle_check_robot_status():
    """Check current robot connection status"""
    socketio.emit("robot_status", {
        "connected": foxglove_bridge.connected,
        "ip": foxglove_bridge.pi_ip
    })

@socketio.on("send_command")
def handle_send_command(data):
    if not foxglove_bridge.connected:
        socketio.emit("log", "⚠️ Not connected to robot!")
        return
    
    cmd = data.get("command", "stop")
    
    # Convert command to Twist velocities
    if cmd == "move up":
        twist = {"linear_x": 0.5, "linear_y": 0.0, "linear_z": 0.0, 
                "angular_x": 0.0, "angular_y": 0.0, "angular_z": 0.0}
    elif cmd == "move down":
        twist = {"linear_x": -0.5, "linear_y": 0.0, "linear_z": 0.0,
                "angular_x": 0.0, "angular_y": 0.0, "angular_z": 0.0}
    elif cmd == "move left":
        twist = {"linear_x": 0.0, "linear_y": 0.0, "linear_z": 0.0,
                "angular_x": 0.0, "angular_y": 0.0, "angular_z": 0.5}
    elif cmd == "move right":
        twist = {"linear_x": 0.0, "linear_y": 0.0, "linear_z": 0.0,
                "angular_x": 0.0, "angular_y": 0.0, "angular_z": -0.5}
    else:  # stop
        twist = {"linear_x": 0.0, "linear_y": 0.0, "linear_z": 0.0,
                "angular_x": 0.0, "angular_y": 0.0, "angular_z": 0.0}

    socketio.emit("log", f"UI command: {cmd}")
    
    # Send TwistStamped command
    if foxglove_bridge.connection_loop:
        asyncio.run_coroutine_threadsafe(
            foxglove_bridge.send_twist_command_once(**twist),
            foxglove_bridge.connection_loop
        )
    else:
        socketio.emit("log", "⚠️ No connection loop available!")


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
