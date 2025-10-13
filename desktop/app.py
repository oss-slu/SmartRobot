from flask import Flask, render_template, jsonify
from flask_socketio import SocketIO, emit
import asyncio
import json
import threading
import time
import websockets
import struct, time

app = Flask(__name__)
app.config['SECRET_KEY'] = 'your-secret-key'
socketio = SocketIO(app, cors_allowed_origins="*")

robot_status = "OFF"

def encode_ros2_string_cdr(s: str) -> bytes:
    # ROS 2 Fast-CDR string = uint32 length INCLUDING null + bytes + '\0'
    b = s.encode("utf-8")
    return struct.pack("<I", len(b) + 1) + b + b"\x00"  # Add the null terminator

async def client_advertise_string_cmd(ws, channel_id: int, topic="/command"):
    # Advertise a client-publish channel for std_msgs/msg/String over CDR
    msg = {
        "op": "advertise", 
        "channels": [{
            "id": channel_id,
            "topic": topic,
            "encoding": "cdr",
            "schemaName": "std_msgs/msg/String",
            "schema": "string data\n"
        }]
    }
    await ws.send(json.dumps(msg))
class FoxgloveBridge:
    def __init__(self):
        self.websocket = None
        self.pi_ip = "10.178.43.25" #always change--> check it!
        self.port = 8765
        self.running = False
        self.connected = False
        self.subscribed_topics = {}
        self.command_channel_id = 4
        
        #should add more topics here to subscribe to
        self.topics_to_subscribe = [
            "/test_talker"
        ]

    def extract_message_from_hex(self, hex_string):
        """Extract message from Foxglove hex data"""
        try:
            # Skip the CDR header (first 32 characters = 16 bytes)
            # The actual message starts after the header
            message_hex = hex_string[32:]  # Skip first 32 hex chars
            
            # Convert hex to string
            message_bytes = bytes.fromhex(message_hex)
            message = message_bytes.decode('utf-8').strip('\x00')
            return message
        except:
            return f"Raw: {hex_string[:50]}..."
        
        
    async def connect_to_pi_server(self):
        """Connect to Foxglove WebSocket server on Raspberry Pi"""
        try:
            socketio.emit('log', f"Attempting to connect to {self.pi_ip}:{self.port}...")
            
            # Connect with Foxglove subprotocol
            self.websocket = await websockets.connect(
                f"ws://{self.pi_ip}:{self.port}", 
                subprotocols=["foxglove.websocket.v1"]
            )
            
            if self.websocket is None:
                raise Exception("WebSocket connection failed - websocket is None")
          
            socketio.emit('log', f"✓ WebSocket CONNECTED with Foxglove protocol")
            self.running = True
            self.connected = True
            
            # Start command sender in background
            asyncio.create_task(self.start_command_sender())
        
            # Handle incoming messages (including advertise and message events)
            await self.handle_messages()
                    
        except Exception as e:
            error_msg = f"✗ WebSocket ERROR - Connection failed: {e}"
            print(error_msg)
            socketio.emit('log', error_msg)
            self.connected = False
        finally:
            self.running = False
            self.connected = False
            if self.websocket:
                await self.websocket.close()
                self.websocket = None
                socketio.emit('log', "✗ WebSocket DISCONNECTED")

    async def handle_messages(self):
        """Handle incoming messages from Foxglove server"""
        if not self.connected or self.websocket is None:
            socketio.emit('log', "⚠️ Cannot handle messages - not connected")
            return
            
        try:
            async for message in self.websocket:
                if self.running and self.connected and self.websocket is not None:
                    try:
                        if isinstance(message, str):
                            # Handle text-based protocol messages
                            data = json.loads(message)
                            
                            # Handle "advertise" message (available topics)
                            if data.get("op") == "advertise" or "channels" in data:
                                channels = data.get("channels", data.get("advertise", []))
                                socketio.emit('log', f"📋 Advertised topics: {len(channels)}")
                                
                                for ch in channels:
                                    topic = ch.get("topic", "unknown")
                                    channel_id = ch.get("id", "unknown")
                                    encoding = ch.get("encoding", "unknown")
                                    
                                    socketio.emit('log', f"Topic: {topic} (Channel: {channel_id}, Encoding: {encoding})")
                                    
                                    # Subscribe to topics we want
                                    if topic in self.topics_to_subscribe:
                                        await self.subscribe_to_channel(channel_id, topic)
                            
                            # Handle other protocol messages
                            else:
                                socketio.emit('log', f"→ Protocol message: {data}")
                                
                        elif isinstance(message, bytes):
                            # Handle binary message data - extract actual message
                            hex_data = message.hex()
                            decoded_message = self.extract_message_from_hex(hex_data)
                            
                            socketio.emit('ros_message', {"data": decoded_message})
                            socketio.emit('log', f"→ Message: {decoded_message}")
                        
                    except json.JSONDecodeError as e:
                        socketio.emit('log', f"⚠️ JSON decode error: {e}")
                    except Exception as e:
                        socketio.emit('log', f"⚠️ Message processing error: {e}")
                else:
                    break
                    
        except Exception as e:
            socketio.emit('log', f"⚠️ Message handling error: {e}")
            self.connected = False

    async def subscribe_to_channel(self, channel_id, topic):
        """Subscribe to a specific channel"""
        try:
            # Generate a unique subscription ID
            subscription_id = len(self.subscribed_topics) + 1
            
            subscribe_msg = {
                "op": "subscribe",
                "subscriptions": [{
                    "id": subscription_id,
                    "channelId": channel_id,
                    "topic": topic
                }]
            }
            
            await self.websocket.send(json.dumps(subscribe_msg))
            
            # Store the mapping between subscription ID and topic
            self.subscribed_topics[subscription_id] = topic
            
            socketio.emit('log', f"✓ Subscribed to: {topic} (subId={subscription_id}, channelId={channel_id})")
            
        except Exception as e:
            socketio.emit('log', f"⚠️ Error subscribing to {topic}: {e}")


    async def start_command_sender(self):
        try:
            # Advertise the command channel
            await client_advertise_string_cmd(self.websocket, self.command_channel_id, "/command")
            socketio.emit("log", f"✓ ClientAdvertised /command (channel {self.command_channel_id})")

            # Wait a moment for advertisement to be processed
            await asyncio.sleep(2)

            seq = 0
            while self.running and self.connected:
                # Try a simpler message format
                payload = encode_ros2_string_cdr("run")
                now_ns = int(time.time() * 1e9)

                # Simplified frame format
                frame = (
                    # b"\x01"  # MESSAGE_DATA opcode
                    # + struct.pack("<I", self.command_channel_id)  # channel ID
                    # + struct.pack("<Q", now_ns)  # timestamp
                    # + struct.pack("<I", seq)  # sequence
                    # + payload  # CDR data
                    b"\x01"  # MESSAGE_DATA opcode
                    + struct.pack("<I", self.command_channel_id)  # channel ID only
                    + payload  # CDR data directly
                )

                await self.websocket.send(frame)
                seq += 1
                socketio.emit("log", f"✓ Sent 'run' (seq={seq-1}) to /command")
                socketio.emit("log", f"Sending binary? {isinstance(frame, bytes)}; frame length={len(frame)}; header={frame[:17].hex()}")

                # Wait longer to see if it works
                await asyncio.sleep(3)  
                
        except Exception as e:
            socketio.emit("log", f"✗ Error in command sender: {e}")

# Global bridge instance
foxglove_bridge = FoxgloveBridge()

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

# @socketio.on('connect')
# def handle_connect():
#     print('Web client connected')
#     emit('log', '✓ Connected to Flask server')

# @socketio.on('disconnect')
# def handle_disconnect():
#     print('Web client disconnected')
#     emit('log', '✗ Disconnected from server')

# @socketio.on('send_command')
# def handle_command(data):
#     command = data['command']
#     print(f"Received command from browser: {command}")
    
#     # Create new event loop for this thread
#     loop = asyncio.new_event_loop()
#     asyncio.set_event_loop(loop)
    
#     try:
#         loop.run_until_complete(foxglove_bridge.send_command_to_pi(command))
#     except Exception as e:
#         print(f"Error sending command: {e}")
#         socketio.emit('log', f"✗ Error sending command: {e}")
#     finally:
#         loop.close()

def start_foxglove_client():
    """Start the Foxglove client in background"""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    
    try:
        loop.run_until_complete(foxglove_bridge.connect_to_pi_server())
    except Exception as e:
        print(f"Error starting Foxglove client: {e}")
        socketio.emit('log', f"Error starting Foxglove client: {e}")
    finally:
        loop.close()

if __name__ == "__main__":
    client_thread = threading.Thread(target=start_foxglove_client, daemon=True)
    client_thread.start()
    
    socketio.run(app, host="0.0.0.0", port=8080, debug=True)