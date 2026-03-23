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

// ── Robot geometry config (mirrors robot_params.yaml) ─────────────────────────

const CFG = {
  scale:       0.02,   // Three.js units per mm  (1000 mm → 20 units)
  xMax:        1050,   // mm
  zMax:        210,    // mm
  stackX:      0,      // mm — carriage X when accessing queue stack (at rail start)
  turntableX:  1050,   // mm — carriage X when accessing turntable (at rail end = xMax)
  safeParkX:   500,    // mm — safe_park.x_mm
  platZ:       45,     // mm — turntable.z_platter_mm
  slotZ:       [20, 60, 100, 140, 180],  // mm — queue_stack.slot_z_mm
  gripOpenUs:  2000,   // µs
  gripCloseUs: 1000,   // µs
  flipAUs:     1500,   // µs — side A
  flipBUs:     500,    // µs — side B
  recordR:     152,    // mm — radius of 12" vinyl
};

// ── Three.js state ─────────────────────────────────────────────────────────────

let renderer3d  = null;
let scene3d     = null;
let camera3d    = null;
let controls3d  = null;
let sceneReady  = false;

// Mesh references updated on every /motion/status message
let arm3d       = null;   // Group — moves along X rail
let zCarriage3d = null;   // Mesh inside arm3d — moves along Y (Z axis)
let aGroup3d    = null;   // Group inside zCarriage3d — rotates around Y (A axis)
let finger1_3d  = null;   // Gripper finger (updated from pincher TPDO)
let finger2_3d  = null;   // Gripper finger (updated from pincher TPDO)
let heldRec3d   = null;   // Record disc held by gripper

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

  // /canopen/pincher/tpdo1 — bytes[0-1]=S1 grip µs, bytes[2-3]=S2 flip µs (uint16 LE)
  new ROSLIB.Topic({ ros, name: '/canopen/pincher/tpdo1',
    messageType: 'std_msgs/msg/UInt8MultiArray' })
  .subscribe(msg => {
    const b = msg.data;
    if (!b || b.length < 4) return;
    const s1 = b[0] | (b[1] << 8);
    const s2 = b[2] | (b[3] << 8);
    updateGripper(s1, s2);
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
  update3DFromStatus(msg);
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

// ── Three.js scene ────────────────────────────────────────────────────────────

function initScene() {
  const canvas = document.getElementById('robot-canvas');
  if (!canvas || typeof THREE === 'undefined') return;

  renderer3d = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer3d.setPixelRatio(window.devicePixelRatio);
  renderer3d.setClearColor(0x0a0a0a);

  scene3d = new THREE.Scene();

  camera3d = new THREE.PerspectiveCamera(50, 1, 0.1, 200);
  camera3d.position.set(10, 14, 18);
  camera3d.lookAt(10, 2, 0);

  controls3d = new THREE.OrbitControls(camera3d, renderer3d.domElement);
  controls3d.target.set(10, 2, 0);
  controls3d.enableDamping = true;
  controls3d.dampingFactor = 0.1;
  controls3d.update();

  const matRail    = new THREE.MeshLambertMaterial({ color: 0x444444 });
  const matCarr    = new THREE.MeshLambertMaterial({ color: 0x888888 });
  const matArm     = new THREE.MeshLambertMaterial({ color: 0xff9900 });
  const matFinger  = new THREE.MeshLambertMaterial({ color: 0xcccccc });
  const matRecord  = new THREE.MeshLambertMaterial({ color: 0x222222, side: THREE.DoubleSide });
  const matPlatter = new THREE.MeshLambertMaterial({ color: 0x333333 });
  const matSpindle = new THREE.MeshLambertMaterial({ color: 0xff9900 });
  const matSlot    = new THREE.MeshLambertMaterial({ color: 0x555555, transparent: true, opacity: 0.6 });

  // X rail
  const railW = CFG.xMax * CFG.scale;  // used by station placement below too
  const rail = new THREE.Mesh(new THREE.BoxGeometry(railW, 0.3, 0.5), matRail);
  rail.position.set(railW / 2, 0, 0);
  scene3d.add(rail);

  // The shoulder arm extends from the carriage in +Z. A-axis rotation swings it
  // ±90° in the horizontal plane to point along ±X and reach the stations, which
  // sit beyond the ends of the rail:
  //   stack    at x = -(shoulderLen)        — carriage at X=0,    A=-90°
  //   turntable at x = railW + shoulderLen  — carriage at X=xMax, A=+90°
  const shoulderLen = 4.5;  // units — must match shoulder geometry below

  // Stack housing — beyond the left end of the rail in X
  const stackWX = -shoulderLen;
  const stackH = new THREE.Mesh(new THREE.BoxGeometry(0.8, 4.2, 0.8), matRail);
  stackH.position.set(stackWX, 2.1, 0);
  scene3d.add(stackH);

  // Slot discs — horizontal records stacked one above the other
  CFG.slotZ.forEach(z_mm => {
    const disc = new THREE.Mesh(
      new THREE.CylinderGeometry(CFG.recordR * CFG.scale, CFG.recordR * CFG.scale, 0.08, 32),
      matSlot
    );
    disc.position.set(stackWX, z_mm * CFG.scale, 0);
    scene3d.add(disc);
  });

  // Turntable platter — beyond the right end of the rail in X
  const ttWX = railW + shoulderLen;
  const ttY = CFG.platZ * CFG.scale;
  const platter = new THREE.Mesh(
    new THREE.CylinderGeometry(CFG.recordR * CFG.scale, CFG.recordR * CFG.scale, 0.3, 48),
    matPlatter
  );
  platter.position.set(ttWX, ttY, 0);
  scene3d.add(platter);
  const spindle = new THREE.Mesh(new THREE.CylinderGeometry(0.15, 0.15, 0.6, 16), matSpindle);
  spindle.position.set(ttWX, ttY + 0.45, 0);
  scene3d.add(spindle);

  // Arm group (moves with X position)
  arm3d = new THREE.Group();
  scene3d.add(arm3d);

  // Vertical column (represents full Z travel range)
  const colH = CFG.zMax * CFG.scale;
  const col = new THREE.Mesh(new THREE.BoxGeometry(0.3, colH, 0.3), matCarr);
  col.position.set(0, colH / 2, 0);
  arm3d.add(col);

  // Z carriage (moves up the column)
  zCarriage3d = new THREE.Mesh(new THREE.BoxGeometry(0.7, 0.7, 0.7), matCarr);
  zCarriage3d.position.set(0, 0, 0);
  arm3d.add(zCarriage3d);

  // A rotation group (rotates around Y — horizontal swing)
  aGroup3d = new THREE.Group();
  zCarriage3d.add(aGroup3d);

  // Shoulder — extends in +Z (toward stations). A rotation swings it left/right/back.
  const shoulder = new THREE.Mesh(new THREE.BoxGeometry(0.35, 0.35, shoulderLen), matArm);
  shoulder.position.set(0, 0, shoulderLen / 2);
  aGroup3d.add(shoulder);

  // Gripper fingers — horizontal arms extending ±X from the shoulder tip.
  // The shoulder tip is at local z = shoulderLen. Fingers spread along X.
  // S1 closed (1000µs): finger inner edge at x=0 (center), outer tip at record circumference.
  // S1 open (2000µs): both fingers slide further out in ±X beyond the record edge.
  const rScale = CFG.recordR * CFG.scale;   // ≈ 3.04 units
  finger1_3d = new THREE.Mesh(new THREE.BoxGeometry(rScale, 0.25, 0.3), matFinger);
  finger1_3d.position.set(rScale / 2, 0, shoulderLen);
  aGroup3d.add(finger1_3d);

  finger2_3d = new THREE.Mesh(new THREE.BoxGeometry(rScale, 0.25, 0.3), matFinger);
  finger2_3d.position.set(-rScale / 2, 0, shoulderLen);
  aGroup3d.add(finger2_3d);

  // Held record — centered at shoulder tip, flips around X axis (axis between grip points)
  heldRec3d = new THREE.Mesh(
    new THREE.CylinderGeometry(rScale, rScale, 0.15, 48),
    matRecord
  );
  heldRec3d.position.set(0, 0, shoulderLen);
  heldRec3d.visible = false;
  aGroup3d.add(heldRec3d);

  // Lights
  scene3d.add(new THREE.AmbientLight(0xffffff, 0.5));
  const dir = new THREE.DirectionalLight(0xffffff, 0.8);
  dir.position.set(15, 20, 10);
  scene3d.add(dir);

  // Grid floor — spans rail + both station overhangs
  const grid = new THREE.GridHelper(40, 40, 0x222222, 0x222222);
  grid.position.set(railW / 2, -0.16, 0);
  scene3d.add(grid);

  _resize3D();
  new ResizeObserver(_resize3D).observe(canvas);
  sceneReady = true;
}

function _resize3D() {
  const canvas = document.getElementById('robot-canvas');
  if (!canvas || !renderer3d) return;
  const w = canvas.clientWidth;
  const h = canvas.clientHeight || 420;
  renderer3d.setSize(w, h, false);
  if (camera3d) { camera3d.aspect = w / h; camera3d.updateProjectionMatrix(); }
}

function animate3D() {
  requestAnimationFrame(animate3D);
  if (!sceneReady) return;
  controls3d.update();
  renderer3d.render(scene3d, camera3d);
}

function update3DFromStatus(msg) {
  if (!sceneReady) return;
  arm3d.position.x       = msg.x_mm * CFG.scale;
  zCarriage3d.position.y = msg.z_mm * CFG.scale;
  aGroup3d.rotation.y    = Math.PI - msg.a_deg * Math.PI / 180;
  // Show held record when arm is between stack and turntable (carrying heuristic)
  heldRec3d.visible = msg.x_mm > CFG.stackX + 50 && msg.x_mm < CFG.turntableX - 50;
}

function updateGripper(s1, s2) {
  if (!sceneReady) return;
  // S1: grip servo. Closed (1000µs) → arms at record edge. Open (2000µs) → arms spread outward.
  const gripT   = Math.max(0, Math.min(1, (s1 - CFG.gripCloseUs) / (CFG.gripOpenUs - CFG.gripCloseUs)));
  const rHalf   = CFG.recordR * CFG.scale / 2;
  const extra   = gripT * 0.8;   // up to 0.8 units beyond record edge when fully open
  finger1_3d.position.x =  rHalf + extra;
  finger2_3d.position.x = -(rHalf + extra);
  // S2: flip servo. Side A (1500µs) → 0°. Side B (500µs) → 180°.
  const flipT = Math.max(0, Math.min(1, (s2 - CFG.flipBUs) / (CFG.flipAUs - CFG.flipBUs)));
  heldRec3d.rotation.x = flipT * Math.PI;
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

window.addEventListener('load', () => {
  if (typeof THREE !== 'undefined') {
    initScene();
    animate3D();
  } else {
    log('Three.js not loaded — 3D view unavailable');
  }
});
