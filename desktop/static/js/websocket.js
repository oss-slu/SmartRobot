const logEl = document.getElementById("ros-log");
const log = (msg) => { 
  console.log(msg); 
  logEl.textContent += msg + "\n"; 
};

// Import Foxglove client
import { FoxgloveClient } from "https://esm.sh/@foxglove/ws-protocol";

const piIP = "10.178.41.99";
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
  log(`Advertised topics: ${channels.length}`);
  for (const ch of channels) {
    log(`Topic: ${ch.topic} (Channel: ${ch.id})`);
    if (ch.topic === "/test_talker") {
      // Subscribe correctly (number, not array)
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
    const reader = readers.get(msg.subscriptionId);
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