"""
This module holds the setup details and constants that define 
how the app communicates with the Foxglove bridge server on the Raspberry Pi
"""

# ---------------------------
# Foxglove Protocol Constants
# ---------------------------

# Client->Server binary frame structure:
#   [0]    : 0x01 (MESSAGE_DATA opcode)
#   [1..4] : channelId (uint32, little-endian)
#   [5.. ] : payload bytes (JSON-encoded for our use case)
MESSAGE_DATA_OPCODE = b"\x01"

# Server->Client binary frame structure:
#   Header = opcode(1 byte) + channelId(4 bytes, uint32 LE) + timestamp(8 bytes, uint64 LE)
#   Total header length = 13 bytes
SERVER_FRAME_HEADER_LEN = 1 + 4 + 8

# ---------------------------
# ROS2 Message Schema Definitions
# ---------------------------

# Geometry_msgs/msg/TwistStamped schema for JSON encoding
TWIST_STAMPED_SCHEMA = (
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
)

# ---------------------------
# Message Encoding Types
# ---------------------------
ENCODING_JSON = "json"
ENCODING_CDR = "cdr"
SCHEMA_ENCODING_ROS2MSG = "ros2msg"

