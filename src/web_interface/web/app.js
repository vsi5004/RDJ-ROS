// RDJ Vinyl Robot — Web UI
// Connects to rosbridge_websocket at ws://<host>:9090

const WS_PORT = 9090;
const wsUrl = `ws://${window.location.hostname}:${WS_PORT}`;

document.getElementById('ws-url').textContent = wsUrl;

let ros = null;
let connected = false;
let isHomed = false;
let isEstop = false;

// Current axis positions tracked from /motion/status
let currentX = 0.0, currentZ = 0.0, currentA = 0.0;

// Per-servo current S1/S2 values (so we only change one channel at a time)
const servoState = {
  pincher: { s1: 1500, s2: 1500 },
  player:  { s1: 1500, s2: 1500 },
};

// ROS objects created on connect
let jogPub       = null;
let servoRawPub  = null;
let cmdPub       = null;
let estopPub     = null;

// Software e-stop state (tracked client-side for immediate UI feedback)
let swEstopActive = false;

// ── Logging ───────────────────────────────────────────────────────────────────

function log(msg) {
  const el = document.getElementById('log');
  const line = document.createElement('div');
  line.textContent = `${new Date().toLocaleTimeString()} ${msg}`;
  el.appendChild(line);
  el.scrollTop = el.scrollHeight;
  if (el.children.length > 100) el.removeChild(el.firstChild);
}

// ── Connection ────────────────────────────────────────────────────────────────

function setConnected(state) {
  connected = state;
  const el = document.getElementById('conn-status');
  el.textContent = state ? '● Connected' : '● Disconnected';
  el.className = state ? 'connected' : 'disconnected';
  if (!state) {
    isHomed = false;
    isEstop = false;
    updateEstopUI();
  }
}

function connect() {
  ros = new ROSLIB.Ros({ url: wsUrl });
  ros.on('connection', () => { setConnected(true);  log('Connected to rosbridge'); subscribeTopics(); });
  ros.on('error',      () => { log('WebSocket error'); });
  ros.on('close',      () => { setConnected(false); log('Disconnected — reconnecting in 3s…'); setTimeout(connect, 3000); });
}

// ── Topics & action clients ────────────────────────────────────────────────────

function subscribeTopics() {
  // Publisher for manual jog commands (avoids ROSLIB.ActionClient ROS 2 compat issues)
  jogPub = new ROSLIB.Topic({
    ros,
    name:        '/motion/jog',
    messageType: 'std_msgs/msg/String',
  });

  // Publisher for raw servo commands
  servoRawPub = new ROSLIB.Topic({
    ros,
    name:        '/motion/servo_raw',
    messageType: 'std_msgs/msg/String',
  });

  // Publisher for high-level commands
  cmdPub = new ROSLIB.Topic({
    ros,
    name:        '/user/command',
    messageType: 'std_msgs/msg/String',
  });

  // Publisher for software e-stop
  estopPub = new ROSLIB.Topic({
    ros,
    name:        '/user/estop',
    messageType: 'std_msgs/msg/Bool',
  });

  // /motion/status — primary telemetry feed
  new ROSLIB.Topic({ ros, name: '/motion/status',
    messageType: 'vinyl_robot_msgs/msg/MotionStatus' })
  .subscribe(onMotionStatus);

  // /turntable/progress
  new ROSLIB.Topic({ ros, name: '/turntable/progress',
    messageType: 'std_msgs/msg/Float32' })
  .subscribe(msg => {
    const pct = msg.data;
    document.getElementById('progress-bar').value = pct;
    document.getElementById('progress-val').textContent = (pct * 100).toFixed(0) + '%';
  });

  // /safety/estop
  new ROSLIB.Topic({ ros, name: '/safety/estop',
    messageType: 'std_msgs/msg/Bool' })
  .subscribe(msg => {
    isEstop = msg.data;
    const el = document.getElementById('estop-indicator');
    el.textContent = isEstop ? '● ACTIVE' : '○ OK';
    el.className = 'status-value ' + (isEstop ? 'bad' : 'ok');
    updateEstopUI();
    if (isEstop) log('⚠ Hardware E-STOP active');
  });
}

// ── /motion/status handler ────────────────────────────────────────────────────

function onMotionStatus(msg) {
  currentX = msg.x_mm;
  currentZ = msg.z_mm;
  currentA = msg.a_deg;
  isHomed  = msg.all_homed;

  // Axis positions
  document.getElementById('x-pos').textContent = msg.x_mm.toFixed(1);
  document.getElementById('z-pos').textContent = msg.z_mm.toFixed(1);
  document.getElementById('a-pos').textContent = msg.a_deg.toFixed(1);

  // Jog position displays (mirror of above)
  document.getElementById('jog-x').textContent = msg.x_mm.toFixed(1);
  document.getElementById('jog-z').textContent = msg.z_mm.toFixed(1);
  document.getElementById('jog-a').textContent = msg.a_deg.toFixed(1);

  // Position snippet for copy-paste into robot_params.yaml
  document.getElementById('pos-snippet').textContent =
    `x_mm: ${msg.x_mm.toFixed(1)}    z_mm: ${msg.z_mm.toFixed(1)}    a_deg: ${msg.a_deg.toFixed(1)}`;

  // Per-axis homed flags
  const homedParts = [
    msg.x_homed ? '<span class="ok">X✓</span>' : '<span style="color:#666">X✗</span>',
    msg.z_homed ? '<span class="ok">Z✓</span>' : '<span style="color:#666">Z✗</span>',
    msg.a_homed ? '<span class="ok">A✓</span>' : '<span style="color:#666">A✗</span>',
  ];
  document.getElementById('homed-flags').innerHTML = homedParts.join(' ');

  // Safety scale
  const scale = msg.velocity_scale;
  const scaleEl = document.getElementById('safety-scale');
  scaleEl.textContent = (scale * 100).toFixed(0) + '%';
  scaleEl.className = 'status-value ' + (scale >= 1.0 ? 'ok' : scale > 0 ? 'warn' : 'bad');

  // ToF readings (65535 = not available)
  function fmtTof(v) { return v > 9000 ? '—' : v.toFixed(0) + ' mm'; }
  document.getElementById('x-tof').textContent       = fmtTof(msg.x_tof_mm);
  document.getElementById('z-tof').textContent       = fmtTof(msg.z_tof_mm);
  document.getElementById('pincher-tof').textContent = fmtTof(msg.pincher_tof_mm);

  // Moving indicators
  const movParts = [
    `<span class="dot ${msg.x_moving ? 'moving' : ''}"></span>X`,
    `<span class="dot ${msg.z_moving ? 'moving' : ''}"></span>Z`,
    `<span class="dot ${msg.a_moving ? 'moving' : ''}"></span>A`,
  ];
  document.getElementById('moving-flags').innerHTML = movParts.join(' ');

  // Fault banner
  const faultBanner = document.getElementById('fault-banner');
  if (msg.fault) {
    faultBanner.style.display = 'block';
    document.getElementById('fault-msg').textContent = msg.fault_msg || 'unknown fault';
  } else {
    faultBanner.style.display = 'none';
  }

  updateJogEnabled();
}

// ── Jog buttons ───────────────────────────────────────────────────────────────

function updateJogEnabled() {
  const disabled = !connected || !isHomed || isEstop;
  document.querySelectorAll('.jog-btn').forEach(b => b.disabled = disabled);
}

function jogAxis(axis, delta) {
  if (!connected) { log('Not connected'); return; }
  if (!isHomed)   { log('Not homed — home first'); return; }
  if (isEstop)    { log('E-stop active'); return; }
  jogPub.publish(new ROSLIB.Message({ data: `${axis} ${delta}` }));
  log(`Jog ${axis.toUpperCase()} ${delta > 0 ? '+' : ''}${delta}`);
}

// ── Servo sliders ─────────────────────────────────────────────────────────────

function onServoSlider(node, channel, value) {
  const us = parseInt(value);
  document.getElementById(`v-${node}-${channel}`).textContent = us;
  servoState[node][channel] = us;
  if (!connected) return;
  servoRawPub.publish(new ROSLIB.Message({
    data: `${node} ${servoState[node].s1} ${servoState[node].s2}`,
  }));
}

// ── E-Stop ────────────────────────────────────────────────────────────────────

function triggerEstop() {
  swEstopActive = true;
  updateEstopUI();
  if (!connected) { log('E-STOP (not connected — no ROS command sent)'); return; }
  estopPub.publish(new ROSLIB.Message({ data: true }));
  log('⛔ E-STOP activated');
}

function clearEstop() {
  if (isEstop && !swEstopActive) {
    log('Cannot clear — hardware E-stop still active'); return;
  }
  swEstopActive = false;
  updateEstopUI();
  if (!connected) return;
  estopPub.publish(new ROSLIB.Message({ data: false }));
  log('E-STOP cleared');
}

function updateEstopUI() {
  const active = swEstopActive || isEstop;
  const btn   = document.getElementById('estop-btn');
  const clear = document.getElementById('clear-estop-btn');
  if (btn)   btn.style.background   = active ? '#f44' : '';
  if (clear) clear.disabled = !active;
  updateJogEnabled();
}

// ── High-level commands ───────────────────────────────────────────────────────

function sendCmd(cmd) {
  if (!connected) { log('Not connected'); return; }
  cmdPub.publish(new ROSLIB.Message({ data: cmd }));
  log(`Command: ${cmd}`);
}

// ── Keyboard shortcuts ────────────────────────────────────────────────────────

document.addEventListener('keydown', e => {
  if (e.key === 'Escape') {
    e.preventDefault();
    triggerEstop();
  }
});

// ── Start ─────────────────────────────────────────────────────────────────────

connect();
