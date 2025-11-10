// desktop/static/js/websocket.js
const socket = io();
let connected = false; // This will be updated when we check status

// UI Elements
const connectBtn = document.getElementById('connect-btn');
const disconnectBtn = document.getElementById('disconnect-btn');
const robotIp = document.getElementById('robot-ip');
const logEl = document.getElementById('activity-log');

// Logging function
function addLog(kind, text) {
  if (!logEl) return; // Safety check
  
  const item = document.createElement('div');
  item.className = 'log-item';
  
  // Create elements and use textContent to safely escape HTML
  const pill = document.createElement('span');
  pill.className = 'pill';
  pill.textContent = kind;
  
  const msg = document.createElement('div');
  msg.className = 'msg';
  msg.textContent = text;
  
  const ts = document.createElement('div');
  ts.className = 'ts';
  ts.textContent = new Date().toLocaleTimeString();
  
  // Append to item
  item.appendChild(pill);
  item.appendChild(msg);
  item.appendChild(ts);
  
  logEl.prepend(item);
}

// Update connection status
function updateStatus(isConnected) {
  connected = isConnected;
  if (connectBtn && disconnectBtn) {
    if (isConnected) {
      connectBtn.disabled = true;
      disconnectBtn.disabled = false;
    } else {
      connectBtn.disabled = false;
      disconnectBtn.disabled = true;
    }
  }
}

// Connect button handler
if (connectBtn) {
  connectBtn.addEventListener('click', () => {
    const ip = robotIp ? robotIp.value : '10.178.43.127';
    socket.emit('connect_robot', { ip: ip });
    addLog('info', `Attempting to connect to ${ip}...`);
  });
}

// Disconnect button handler
if (disconnectBtn) {
  disconnectBtn.addEventListener('click', () => {
    socket.emit('disconnect_robot');
    addLog('info', 'Disconnecting from robot...');
  });
}

// Check robot status when page loads
socket.on('connect', () => {
  addLog('info', 'Connected to Flask server');
  socket.emit('check_robot_status'); // Ask for current status
});

// Handle robot status response
socket.on('robot_status', (data) => {
  if (data.connected) {
    updateStatus(true);
    addLog('success', 'Robot connected!');
  } else {
    updateStatus(false);
    addLog('info', 'Robot not connected');
  }
});

// Handle robot disconnected event
socket.on('robot_disconnected', () => {
  updateStatus(false);
  addLog('info', 'Robot disconnected');
});

// Command buttons
document.querySelectorAll('[data-cmd]').forEach(btn => {
  btn.addEventListener('click', () => {
    if (!connected) {
      addLog('error', 'Not connected to robot!');
      return;
    }
    
    const cmd = btn.getAttribute('data-cmd');
    const speed = document.getElementById('speed') ? document.getElementById('speed').value : 100;
    
    // Send the actual command text
    let commandText = '';
    if (cmd === 'move_up') commandText = 'move up';
    else if (cmd === 'move_down') commandText = 'move down';
    else if (cmd === 'move_left') commandText = 'move left';
    else if (cmd === 'move_right') commandText = 'move right';
    else if (cmd === 'stop') commandText = 'stop';

    socket.emit('send_command', { command: commandText, speed: speed });
  });
});

// Socket event handlers
socket.on('disconnect', () => {
  addLog('warn', 'Disconnected from Flask server');
  updateStatus(false);
});
socket.on('log', (msg) => addLog('info', msg));
socket.on('ros_message', (data) => addLog('ros', JSON.stringify(data)));

// Speed display
const speed = document.getElementById('speed');
const speedVal = document.getElementById('speed-val');
if (speed && speedVal) {
  speed.addEventListener('input', () => {
    speedVal.textContent = speed.value + '%';
  });
}

// Tabs
const tabs = document.querySelectorAll('.tab');
const panels = document.querySelectorAll('.tab-panel');
tabs.forEach(t => t.addEventListener('click', () => {
  tabs.forEach(x => x.classList.remove('active'));
  panels.forEach(p => p.classList.remove('active'));
  t.classList.add('active');
  document.getElementById('panel-' + t.dataset.tab).classList.add('active');
}));

// Initialize
updateStatus(false);
addLog('info', 'WebSocket status monitoring started');
