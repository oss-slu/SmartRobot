"""
Foxglove WebSocket protocol utilities.

This module contains functions for encoding, decoding, and parsing
messages according to the Foxglove WebSocket protocol specification
"""

import json
import struct
from typing import Dict, Any

from .constants import (
    MESSAGE_DATA_OPCODE,
    TWIST_STAMPED_SCHEMA,
    ENCODING_JSON,
    SCHEMA_ENCODING_ROS2MSG
)


def build_json_payload_for_string(data_text: str) -> bytes:
    """
    Build JSON payload for std_msgs/msg/String message.
    
    For std_msgs/msg/String with encoding='json', the payload is:
    {"data":"<your text>"} encoded as UTF-8 bytes.
    
    Args:
        data_text: The string data to encode
        
    Returns:
        UTF-8 encoded JSON bytes
    """
    return json.dumps({"data": data_text}).encode("utf-8")


def parse_cdr_string(payload: bytes) -> str:
    """
    Parse CDR-encoded std_msgs/msg/String payload.
    
    CDR format structure:
    [encapsulation_header(4 bytes)] [string_length(4 bytes)] [string_data]
    
    Args:
        payload: CDR-encoded bytes
        
    Returns:
        Decoded string data
        
    Raises:
        ValueError: If payload is invalid or truncated
    """
    if len(payload) < 8:
        return "<invalid CDR payload>"
    
    # Skip encapsulation header (first 4 bytes)
    # Read string length (little-endian uint32 at offset 4)
    string_length = struct.unpack("<I", payload[4:8])[0]
    
    if len(payload) < 8 + string_length:
        return "<truncated CDR payload>"
    
    # Extract and decode string data
    string_data = payload[8:8+string_length].decode("utf-8", errors="replace")
    return string_data


async def advertise_twist_channel(
    websocket, 
    channel_id: int, 
    topic: str = "/diff_drive_base/cmd_vel"
) -> None:
    """
    Advertise a client-publish channel for geometry_msgs/msg/TwistStamped.
    
    This function sends an advertise message to the Foxglove server indicating
    that this client will publish TwistStamped messages on the specified topic.
    
    Args:
        websocket: The WebSocket connection to send the message on
        channel_id: Unique channel ID for this topic
        topic: ROS2 topic name (default: /diff_drive_base/cmd_vel)
        
    Raises:
        Exception: If WebSocket send fails
    """
    advertise_message = {
        "op": "advertise",
        "channels": [
            {
                "id": channel_id,
                "topic": topic,
                "encoding": ENCODING_JSON,
                "schemaName": "geometry_msgs/msg/TwistStamped",
                "schemaEncoding": SCHEMA_ENCODING_ROS2MSG,
                "schema": TWIST_STAMPED_SCHEMA,
            }
        ],
    }
    await websocket.send(json.dumps(advertise_message))


def build_twist_stamped_frame(
    channel_id: int,
    linear_x: float = 0.0,
    linear_y: float = 0.0,
    linear_z: float = 0.0,
    angular_x: float = 0.0,
    angular_y: float = 0.0,
    angular_z: float = 0.0,
    frame_id: str = "base_link"
) -> bytes:
    """
    Build a binary frame for a TwistStamped message.
    
    Creates a complete Foxglove protocol frame ready to send over WebSocket.
    
    Args:
        channel_id: Channel ID for the command topic
        linear_x: Linear velocity in x direction (m/s)
        linear_y: Linear velocity in y direction (m/s)
        linear_z: Linear velocity in z direction (m/s)
        angular_x: Angular velocity around x axis (rad/s)
        angular_y: Angular velocity around y axis (rad/s)
        angular_z: Angular velocity around z axis (rad/s)
        frame_id: Reference frame ID (default: "base_link")
        
    Returns:
        Complete binary frame ready to send
    """
    # Build TwistStamped message
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
    
    # Encode payload as JSON
    payload = json.dumps(twist_stamped_data).encode("utf-8")
    
    # Build frame: opcode + channel_id + payload
    frame = MESSAGE_DATA_OPCODE + struct.pack("<I", channel_id) + payload
    
    return frame

