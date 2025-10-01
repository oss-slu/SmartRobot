// Simple logging function
const log = (msg) => { 
  const logEl = document.getElementById("ros-log");
  if (logEl) {
    logEl.textContent += msg + "\n"; 
    console.log("Message added to log element");
  } else {
    console.error("Log element NOT found!");
  }
};


import * as Rosmsg from "https://esm.sh/@foxglove/rosmsg";
const parseMsgDef = Rosmsg.parse ?? Rosmsg.default?.parse ?? Rosmsg.parseDefinition;

import { MessageReader, MessageWriter } from "https://esm.sh/@foxglove/rosmsg2-serialization";
import { FoxgloveClient } from "https://esm.sh/@foxglove/ws-protocol";



const logEl = document.getElementById("ros-log");

const piIP = "10.178.40.131"; //PI IP address might change--> check it!
const port = 8765;
const wsUrl = `ws://${piIP}:${port}`;

log(`Attempting to connect to ${wsUrl}...`);

// Create WebSocket with correct subprotocol
const ws = new WebSocket(wsUrl, "foxglove.websocket.v1");

// Create Foxglove client
const client = new FoxgloveClient({ ws });

// WebSocket connection events
ws.onopen = () => {
  log("✓ WebSocket CONNECTED with Foxglove protocol");
};

ws.onclose = (event) => {
  log(`✗ WebSocket DISCONNECTED (Code: ${event.code})`);
  if (event.code === 1002) {
    log("  → Protocol error - subprotocol mismatch");
  }
};

ws.onerror = () => {
  log("✗ WebSocket ERROR - Connection failed");
};

const readers = new Map();
const topicsBySub = new Map();

client.on("advertise", (channels) => {
  //list advertised topics
  log(`Advertised topics: ${channels.length}`);
  //loop through topics
  for (const ch of channels) {
    log(`Topic: ${ch.topic} (Channel: ${ch.id})`);
    // Subscribe only to /test_talker -->
    //in the future we will have more topics like
    //  /sensor_data, /robot_status, /camera1/image_raw,...
    if (ch.topic === "/test_talker") {
      // Subscribe
      const subId = client.subscribe(ch.id);
      topicsBySub.set(subId, ch.topic);
      // Build a CDR reader for this topic
      if (ch.encoding === "cdr" && (ch.schemaEncoding === "ros2msg" || ch.schemaEncoding === "ros2idl")) {
        const defs = parseMsgDef(ch.schema, { ros2: true });
        const reader = new MessageReader(defs);
        readers.set(subId, reader);
      }
      log(`✓ Subscribed to: ${ch.topic} (subId=${subId})`);
    }
  }
});


client.on("message", (msg) => {
  //get reader for this message subscription
    const reader = readers.get(msg.subscriptionId);
    //if we have a reader (which mean we subscribed to this topic), decode the message
    if (reader) {
      const decoded = reader.readMessage(msg.data);
      log(`  → Decoded message: ${JSON.stringify(decoded)}`);
    } else {  
      // No reader available, try to show as text
    try {
      const text = new TextDecoder().decode(msg.data);
      log(`  → Raw message: ${text}`);
    } catch (error) {
      log(`  → Cannot decode message: ${error.message}`);
    }
    }
});

log("WebSocket status monitoring started");
// // send command to robot 
// // send "RUN" or "STOP" to /web_cmd
// function sendCommand(text) {
//   const schema = { name:"std_msgs/String", encoding:"ros2msg", data:"string data\n" };
//   client.advertise([{ id: 3, topic: "/test_talker", schema }]);
//   const writer = new MessageWriter(schema);
//   const buffer = writer.write({ data: text });
//   client.message({ channelId:3, data: buffer });
//   log(`Sent command: ${text}`);
//   // Update local status
//   fetch(`/set_status/${text === "RUN" ? "ON" : "OFF"}`);
// }

// // Hook buttons
// document.getElementById("run").onclick  = () => sendCommand("RUN");
// document.getElementById("stop").onclick = () => sendCommand("STOP");