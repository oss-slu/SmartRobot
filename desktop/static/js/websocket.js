/*  based on the concept of https://github.com/foxglove/ws-protocol/tree/main/typescript/ws-protocol#client-template */
// FoxgloveClient-> connects to a Foxglove WebSocket server
// MessageWriter->help encode ROS2 messages to send over the WebSocket
import { FoxgloveClient, MessageWriter } from "https://esm.sh/@foxglove/ws-protocol";

const logEl = document.getElementById("ros-log");
const log = msg => logEl.textContent += msg + "\n";

// Connect to Foxglove Bridge on the Pi
const client = new FoxgloveClient({
  ws: new WebSocket("ws://<pi-ip>:8765")  //replace with our Pi IP
  // In the future  we can add BLE support here for multiple ras pi
});
//logs when the WebSocket opens or closes
client.on("open", () => log("Connected"));
client.on("close", () => log("Disconnected"));

//subscribe to topics when advertised by robot
//soon, we will store subscriptions in a map
//so each topic updates a different part of the dashboard
client.on("advertise", channels => {
  //loop through advertised topics
  channels.forEach(ch => {
    log(`Topic: ${ch.topic}`);
    //if the topic is one we want, subscribe
    if(ch.topic === "/chatter" || ch.topic === "/sensor" || ch.topic === "/motor") {
      //subscribe
      client.subscribe([{ channelId: ch.id, encoding: "cdr" }]);
    }
  });
});


//report messages received on subscribed topics
client.on("message", msg => {
  log(`Message on channel ${msg.channelId}, size=${msg.data.byteLength}`);
});

//send command to robot 
//send "RUN" or "STOP" to /web_cmd
function sendCommand(text) {
  const schema = { name:"std_msgs/String", encoding:"ros2msg", data:"string data\n" };
  client.advertise([{ id: 1, topic: "/web_cmd", schema }]);
  const writer = new MessageWriter(schema);
  const buffer = writer.write({ data: text });
  client.message({ channelId:1, data: buffer });
  log(`Sent command: ${text}`);
  // Update local status
  fetch(`/set_status/${text === "RUN" ? "ON" : "OFF"}`);
}

// Hook buttons
document.getElementById("run").onclick  = () => sendCommand("RUN");
document.getElementById("stop").onclick = () => sendCommand("STOP");
