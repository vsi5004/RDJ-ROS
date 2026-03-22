// RDJ Vinyl Robot — Web UI
// Connects to rosbridge_websocket at ws://<host>:9090

const WS_PORT = 9090;
const wsUrl = `ws://${window.location.hostname}:${WS_PORT}`;

document.getElementById('ws-url').textContent = wsUrl;

let ros = null;
let connected = false;

function log(msg) {
  const el = document.getElementById('log');
  const line = document.createElement('div');
  line.textContent = `${new Date().toLocaleTimeString()} ${msg}`;
  el.appendChild(line);
  el.scrollTop = el.scrollHeight;
  if (el.children.length > 50) el.removeChild(el.firstChild);
}

function setConnected(state) {
  connected = state;
  const el = document.getElementById('conn-status');
  el.textContent = state ? '● Connected' : '● Disconnected';
  el.className = state ? 'connected' : 'disconnected';
}

function connect() {
  ros = new ROSLIB.Ros({ url: wsUrl });

  ros.on('connection', () => {
    setConnected(true);
    log('Connected to rosbridge');
    subscribeTopics();
  });

  ros.on('error', (e) => {
    log('WebSocket error: ' + e);
  });

  ros.on('close', () => {
    setConnected(false);
    log('Disconnected — reconnecting in 3s…');
    setTimeout(connect, 3000);
  });
}

function subscribeTopics() {
  // /motion/status
  const motionSub = new ROSLIB.Topic({
    ros,
    name: '/motion/status',
    messageType: 'vinyl_robot_msgs/msg/MotionStatus',
  });
  motionSub.subscribe((msg) => {
    document.getElementById('x-pos').textContent = msg.x_mm.toFixed(1);
    document.getElementById('z-pos').textContent = msg.z_mm.toFixed(1);
    document.getElementById('a-pos').textContent = msg.a_deg.toFixed(1);
    document.getElementById('homed').textContent = msg.all_homed ? '✓' : '✗';
    document.getElementById('safety-scale').textContent =
      (msg.velocity_scale * 100).toFixed(0) + '%';
  });

  // /turntable/progress
  const progressSub = new ROSLIB.Topic({
    ros,
    name: '/turntable/progress',
    messageType: 'std_msgs/msg/Float32',
  });
  progressSub.subscribe((msg) => {
    const pct = msg.data;
    document.getElementById('progress-bar').value = pct;
    document.getElementById('progress-val').textContent = (pct * 100).toFixed(0) + '%';
  });
}

function sendCmd(cmd) {
  if (!connected) { log('Not connected'); return; }
  const pub = new ROSLIB.Topic({
    ros,
    name: '/user/command',
    messageType: 'std_msgs/msg/String',
  });
  pub.publish(new ROSLIB.Message({ data: cmd }));
  log(`Sent command: ${cmd}`);
}

connect();
