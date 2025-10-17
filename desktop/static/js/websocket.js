// Simple SocketIO client
const socket = io();

const log = (msg) => { 
    const logEl = document.getElementById("ros-log");
    if (logEl) {
        logEl.textContent += msg + "\n"; 
        console.log("Message added to log element");
    } else {
        console.error("Log element NOT found!");
    }
};

socket.on('connect', () => {
    log("✓ Connected to Flask server");
});

socket.on('log', (msg) => {
    log(msg);
});

socket.on('ros_message', (data) => {
    log(`→ ROS Message: ${JSON.stringify(data)}`);
});

socket.on('status', (data) => {
    log(`Status: ${data.data}`);
});

socket.on('disconnect', () => {
    log("✗ Disconnected from server");
});

log("WebSocket status monitoring started");


// function sendCommand(command) {
//     socket.emit('send_command', {command: command});
// }

// // Hook to buttons
// document.getElementById("run").onclick = () => sendCommand("RUN");
// document.getElementById("stop").onclick = () => sendCommand("STOP");