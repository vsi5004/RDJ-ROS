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
let jogPub          = null;
let servoRawPub     = null;
let cmdPub          = null;
let estopPub        = null;
let selectSlotPub   = null;
let playModePub     = null;

// Queue and player state
let currentSlotIdx  = -1;
let pendingSlotIdx  = -1;
let currentPlayMode = 'SEQUENTIAL';
let playerHasRecord = false;
let currentSide     = 'A';    // 'A' or 'B' — from /state_machine/current_side
let startSent       = false;  // true after operator presses Start Playing (one-shot)

// Software e-stop state (tracked client-side for immediate UI feedback)
let swEstopActive = false;

// ── Robot geometry config (mirrors robot_params.yaml) ─────────────────────────

const CFG = {
  scale:          0.02,   // Three.js units per mm  (1000 mm → 20 units)
  xMax:           1050,   // mm
  zMax:           210,    // mm
  stackX:         0,      // mm — carriage X when accessing queue stack (at rail start)
  turntableX:     1050,   // mm — carriage X when accessing turntable (at rail end = xMax)
  safeParkX:      500,    // mm — safe_park.x_mm
  platZ:          45,     // mm — turntable.z_platter_mm
  slotZ:          [20, 60, 100, 140, 180],  // mm — queue_stack.slot_z_mm
  shelfClearance: 20,     // mm — homing.record_stack.shelf_clearance_mm (slot envelope height)
  xClearMm:       250,    // mm — homing.record_stack.x_clear_mm (A rotation forbidden below this)
  gripOpenUs:     2000,   // µs
  gripCloseUs:    1000,   // µs
  flipAUs:        1500,   // µs — side A
  flipBUs:        500,    // µs — side B
  recordR:        152,    // mm — radius of 12" vinyl
};

// ── LED Visualizer state ───────────────────────────────────────────────────────

const LED_STRIP_COUNT = 144;
const LED_RING_COUNT  = 60;

let stripRects = [];   // SVG <rect> elements for X-rail strip
let ringArcs   = [];   // SVG <path> elements for A-axis ring

// ── Three.js state ─────────────────────────────────────────────────────────────

let renderer3d  = null;
let scene3d     = null;
let camera3d    = null;
let controls3d  = null;
let sceneReady  = false;

// Mesh references updated on every /motion/status message
let arm3d        = null;   // Group — moves along X rail
let zCarriage3d  = null;   // Mesh inside arm3d — moves along Y (Z axis)
let aGroup3d     = null;   // Group inside zCarriage3d — rotates around Y (A axis)
let finger1_3d   = null;   // Gripper finger (updated from pincher TPDO)
let slotEnvelopes = [];    // Per-slot Z-envelope indicator meshes (5 entries)
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
    const flipBtn  = document.getElementById('flip-btn');
    if (flipBtn)  flipBtn.disabled  = true;
    const startBtn = document.getElementById('start-btn');
    if (startBtn) startBtn.disabled = true;
  }
  _updateStartBtn();
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

  // Publisher for slot override
  selectSlotPub = new ROSLIB.Topic({
    ros,
    name:        '/user/select_record',
    messageType: 'std_msgs/msg/UInt8',
  });

  // Publisher for play mode
  playModePub = new ROSLIB.Topic({
    ros,
    name:        '/user/play_mode',
    messageType: 'std_msgs/msg/String',
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

  // /safety/status — hardware e-stop and velocity scale from lidar_safety
  new ROSLIB.Topic({ ros, name: '/safety/status',
    messageType: 'vinyl_robot_msgs/msg/SafetyStatus' })
  .subscribe(msg => {
    isEstop = msg.estop;
    const el = document.getElementById('estop-indicator');
    el.textContent = isEstop ? '● ACTIVE' : '○ OK';
    el.className = 'status-value ' + (isEstop ? 'bad' : 'ok');
    updateEstopUI();
    if (isEstop) log(`⚠ Hardware E-STOP active (${msg.reason || 'lidar'})`);
  });

  // /canopen/pincher/tpdo1 — bytes[0-1]=S1 grip µs, bytes[2-3]=S2 flip µs (uint16 LE)
  new ROSLIB.Topic({ ros, name: '/canopen/pincher/tpdo1',
    messageType: 'std_msgs/msg/UInt8MultiArray' })
  .subscribe(msg => {
    const b = decodeBytes(msg.data);
    if (!b || b.length < 4) return;
    const s1 = b[0] | (b[1] << 8);
    const s2 = b[2] | (b[3] << 8);
    updateGripper(s1, s2);
  });

  // /led/pixels — per-LED RGB from led_controller at ~20 Hz
  new ROSLIB.Topic({ ros, name: '/led/pixels',
    messageType: 'std_msgs/msg/UInt8MultiArray' })
  .subscribe(onLedPixels);

  // /led/pattern — current semantic LED pattern from state machine
  new ROSLIB.Topic({ ros, name: '/led/pattern',
    messageType: 'std_msgs/msg/String' })
  .subscribe(msg => {
    const el = document.getElementById('led-pattern-label');
    if (el) el.textContent = `pattern: ${msg.data}`;
  });

  // /state_machine/tip — active behaviour tree node name (10 Hz)
  new ROSLIB.Topic({ ros, name: '/state_machine/tip',
    messageType: 'std_msgs/msg/String' })
  .subscribe(msg => {
    const el = document.getElementById('sm-tip');
    if (el) el.textContent = msg.data;
  });

  // /state_machine/status — composite: slot, side, player occupancy, initial_loaded
  new ROSLIB.Topic({ ros, name: '/state_machine/status',
    messageType: 'vinyl_robot_msgs/msg/StateMachineStatus' })
  .subscribe(msg => {
    // Slot index only meaningful once initial_loaded; before that all discs are in slots
    if (msg.initial_loaded) {
      currentSlotIdx = msg.slot_idx;
      if (pendingSlotIdx === currentSlotIdx) pendingSlotIdx = -1;
    }
    updateSlotUI();

    // Disc side drives 3D orientation from state machine knowledge, not servo S2
    currentSide = msg.current_side;
    if (heldRec3d) {
      heldRec3d.rotation.x = (currentSide === 'A') ? Math.PI : 0;
    }

    // Player occupancy — enables Flip Now; hides Start Playing once first disc is loaded
    playerHasRecord = msg.player_has_record;
    const flipBtn = document.getElementById('flip-btn');
    if (flipBtn) flipBtn.disabled = !playerHasRecord || !connected;
    if (playerHasRecord) {
      const startBtn = document.getElementById('start-btn');
      if (startBtn) startBtn.style.display = 'none';
    }
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
  _updateStartBtn();
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

// ── Start Playing ─────────────────────────────────────────────────────────────

function _updateStartBtn() {
  const btn = document.getElementById('start-btn');
  if (!btn) return;
  // Available only when connected, homed, and not yet triggered
  btn.disabled = !connected || !isHomed || startSent || isEstop;
  btn.textContent = startSent ? 'Starting…' : 'Start Playing';
}

function triggerStart() {
  if (!connected || startSent) return;
  startSent = true;
  _updateStartBtn();
  sendCmd('start');
  log('▶ Start Playing — loading slot 0 onto player');
}

// ── High-level commands ───────────────────────────────────────────────────────

function sendCmd(cmd) {
  if (!connected) { log('Not connected'); return; }
  cmdPub.publish(new ROSLIB.Message({ data: cmd }));
  log(`Command: ${cmd}`);
}

// ── Queue slot and play mode controls ─────────────────────────────────────────

function selectSlot(idx) {
  if (!connected) { log('Not connected'); return; }
  pendingSlotIdx = idx;
  updateSlotUI();
  selectSlotPub.publish(new ROSLIB.Message({ data: idx }));
  log(`Queue: override → slot ${idx}`);
}

function setPlayMode(mode) {
  if (!connected) { log('Not connected'); return; }
  currentPlayMode = mode;
  updateModeUI();
  playModePub.publish(new ROSLIB.Message({ data: mode }));
  log(`Play mode: ${mode}`);
}

function updateSlotUI() {
  const QUEUE_SIZE = 5;
  for (let i = 0; i < QUEUE_SIZE; i++) {
    const btn = document.getElementById(`slot-btn-${i}`);
    if (!btn) continue;
    const isCurrent = (i === currentSlotIdx);
    const isPending  = (i === pendingSlotIdx && !isCurrent);
    const isEmpty    = isCurrent;  // that slot's record is out (on player or in gripper)
    btn.classList.toggle('active-slot',  isCurrent);
    btn.classList.toggle('pending-slot', isPending);
    btn.textContent = `Slot ${i} ${isEmpty ? '○' : '●'}`;
  }
  const curEl = document.getElementById('current-slot');
  if (curEl) curEl.textContent = currentSlotIdx >= 0 ? `Slot ${currentSlotIdx}` : '—';
  const ovEl = document.getElementById('override-slot');
  if (ovEl)  ovEl.textContent  = (pendingSlotIdx >= 0 && pendingSlotIdx !== currentSlotIdx)
                                   ? `Slot ${pendingSlotIdx}` : '—';
}

function updateModeUI() {
  ['SEQUENTIAL', 'SIDE_REPEAT', 'SINGLE_REPEAT'].forEach(m => {
    const btn = document.getElementById(`mode-btn-${m}`);
    if (btn) btn.classList.toggle('active-mode', m === currentPlayMode);
  });
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
  const matRecord  = new THREE.MeshLambertMaterial({ color: 0x222222 });
  const matLabelA  = new THREE.MeshLambertMaterial({ color: 0xff6600 }); // orange label — side A
  const matLabelB  = new THREE.MeshLambertMaterial({ color: 0x2255cc }); // blue label — side B
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

  // Stack — no housing box; shelf plates and envelope indicators define the structure
  const stackWX = -shoulderLen;
  const recR = CFG.recordR * CFG.scale;  // ≈ 3.04 units — shelf/envelope radius matches records
  const discH = 0.08;                    // record disc height in scene units

  // Slot discs — rest on shelf plates; center at slot_z + half disc height
  CFG.slotZ.forEach(z_mm => {
    const disc = new THREE.Mesh(
      new THREE.CylinderGeometry(recR, recR, discH, 48),
      matSlot
    );
    disc.position.set(stackWX, z_mm * CFG.scale + discH / 2, 0);
    scene3d.add(disc);
  });

  // Shelf floor plates — one thin plate per slot, top surface at slot_z[i].
  // Records rest on top of these plates. Thin (5mm visual) so the open slot space is clear.
  const shelfThk = 5 * CFG.scale;  // 5mm visual thickness
  const matShelf = new THREE.MeshPhongMaterial({ color: 0x888888, opacity: 0.85, transparent: true });
  CFG.slotZ.forEach(z_mm => {
    const shelf = new THREE.Mesh(
      new THREE.CylinderGeometry(recR, recR, shelfThk, 48),
      matShelf.clone()
    );
    // Top surface at z_mm * scale; center is half-thickness below that
    shelf.position.set(stackWX, z_mm * CFG.scale - shelfThk / 2, 0);
    scene3d.add(shelf);
  });

  // Vertical support pillars flanking the record slots (A rotation hits these when X < x_clear_mm)
  const matPillar = new THREE.MeshPhongMaterial({ color: 0x666666 });
  const pillarH = (CFG.slotZ[CFG.slotZ.length - 1] + CFG.shelfClearance * 2) * CFG.scale;
  [-recR * 0.9, recR * 0.9].forEach(zOff => {
    const pillar = new THREE.Mesh(new THREE.BoxGeometry(0.07, pillarH, 0.07), matPillar);
    pillar.position.set(stackWX, pillarH / 2, zOff);
    scene3d.add(pillar);
  });

  // Per-slot Z-envelope indicators — translucent green bands showing the 20mm safe operating zone.
  // Same radius as records. Highlighted bright green (active, Z in bounds) or red (Z out of bounds).
  slotEnvelopes = CFG.slotZ.map(z_mm => {
    const mat = new THREE.MeshBasicMaterial({ color: 0x00cc44, opacity: 0.18, transparent: true });
    const envH = CFG.shelfClearance * CFG.scale;
    const env = new THREE.Mesh(new THREE.CylinderGeometry(recR, recR, envH, 48), mat);
    env.position.set(stackWX, (z_mm + CFG.shelfClearance / 2) * CFG.scale, 0);
    scene3d.add(env);
    return env;
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

  // U-gripper: crossbar at the arm base + two forward-pointing legs that grip the record edge.
  // Legs move in ±X to open/close. Closed (S1=1000µs): legs at ±rScale, touching disc edge.
  // Open (S1=2000µs): legs spread outward beyond the disc.
  const rScale = CFG.recordR * CFG.scale;   // ≈ 3.04 units

  // Crossbar — fixed horizontal backbone connecting the carriage to the two legs (base of U)
  const crossbar = new THREE.Mesh(new THREE.BoxGeometry(rScale * 2 + 0.7, 0.35, 0.35), matArm);
  crossbar.position.set(0, 0, 0.175);
  aGroup3d.add(crossbar);

  // Right leg of the U — extends forward in +Z, slides in X with grip servo
  finger1_3d = new THREE.Mesh(new THREE.BoxGeometry(0.35, 0.35, shoulderLen), matFinger);
  finger1_3d.position.set(rScale, 0, shoulderLen / 2);
  aGroup3d.add(finger1_3d);

  // Left leg of the U — extends forward in +Z, slides in X with grip servo
  finger2_3d = new THREE.Mesh(new THREE.BoxGeometry(0.35, 0.35, shoulderLen), matFinger);
  finger2_3d.position.set(-rScale, 0, shoulderLen / 2);
  aGroup3d.add(finger2_3d);

  // Held record — centered at shoulder tip, flips around X axis (axis between grip points)
  heldRec3d = new THREE.Mesh(
    new THREE.CylinderGeometry(rScale, rScale, 0.15, 48),
    [matRecord, matLabelB, matLabelA]  // [side, top(+Y=B-up), bottom(-Y=A-up)]
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

  // Slot Z-envelope highlighting — shows safe operating zone when arm is near stack.
  // Bright green = arm Z is physically within this slot's envelope.
  // Dim = arm not near stack or Z is in a different slot.
  const nearStack = msg.x_mm < CFG.xClearMm;
  slotEnvelopes.forEach((env, i) => {
    const slotZ    = CFG.slotZ[i];
    const slotTopZ = slotZ + CFG.shelfClearance;
    const zInBounds = msg.z_mm >= slotZ && msg.z_mm <= slotTopZ;

    if (!nearStack) {
      env.material.opacity = 0.15;
      env.material.color.setHex(0x00cc44);
    } else if (zInBounds) {
      env.material.opacity = 0.55;
      env.material.color.setHex(0x00ee55);   // bright green: arm Z physically in this envelope
    } else {
      env.material.opacity = 0.06;
      env.material.color.setHex(0x00cc44);
    }
    env.material.needsUpdate = true;
  });
}

function updateGripper(s1, _s2) {
  if (!sceneReady) return;
  // S1: grip servo. Closed (1000µs) → arms at record edge. Open (2000µs) → arms spread outward.
  const gripT  = Math.max(0, Math.min(1, (s1 - CFG.gripCloseUs) / (CFG.gripOpenUs - CFG.gripCloseUs)));
  const rScale = CFG.recordR * CFG.scale;
  const extra  = gripT * 0.8;   // up to 0.8 units beyond record edge when fully open
  finger1_3d.position.x =  rScale + extra;
  finger2_3d.position.x = -(rScale + extra);
  heldRec3d.visible = gripT < 0.5;   // disc appears when pinchers are closed
  // Disc orientation is driven by currentSide (from /state_machine/status), not S2,
  // because the mock resets the flip servo after each action — S2 is unreliable.
  heldRec3d.rotation.x = (currentSide === 'A') ? Math.PI : 0;
}

// ── LED Visualizer ────────────────────────────────────────────────────────────

function initLEDVisualizer() {
  const svgNS = 'http://www.w3.org/2000/svg';

  // Strip: 144 rect elements, each 1×4 in the viewBox "0 0 144 4"
  const stripSvg = document.getElementById('led-strip');
  if (stripSvg) {
    for (let i = 0; i < LED_STRIP_COUNT; i++) {
      const rect = document.createElementNS(svgNS, 'rect');
      rect.setAttribute('x', i);
      rect.setAttribute('y', 0);
      rect.setAttribute('width', 1);
      rect.setAttribute('height', 4);
      rect.setAttribute('fill', '#111');
      stripSvg.appendChild(rect);
      stripRects.push(rect);
    }
  }

  // Ring: 60 arc path segments (6° each), SVG arc paths on a unit circle
  const ringSvg = document.getElementById('led-ring');
  if (ringSvg) {
    const outerR = 0.9;
    const innerR = 0.6;
    for (let i = 0; i < LED_RING_COUNT; i++) {
      const a0 = (i * 6 - 90) * Math.PI / 180;       // offset -90° so 0° = 12 o'clock
      const a1 = ((i + 1) * 6 - 90) * Math.PI / 180;
      const ox0 = Math.cos(a0) * outerR, oy0 = Math.sin(a0) * outerR;
      const ox1 = Math.cos(a1) * outerR, oy1 = Math.sin(a1) * outerR;
      const ix1 = Math.cos(a1) * innerR, iy1 = Math.sin(a1) * innerR;
      const ix0 = Math.cos(a0) * innerR, iy0 = Math.sin(a0) * innerR;
      const d = [
        `M ${ox0.toFixed(4)} ${oy0.toFixed(4)}`,
        `A ${outerR} ${outerR} 0 0 1 ${ox1.toFixed(4)} ${oy1.toFixed(4)}`,
        `L ${ix1.toFixed(4)} ${iy1.toFixed(4)}`,
        `A ${innerR} ${innerR} 0 0 0 ${ix0.toFixed(4)} ${iy0.toFixed(4)}`,
        'Z',
      ].join(' ');
      const path = document.createElementNS(svgNS, 'path');
      path.setAttribute('d', d);
      path.setAttribute('fill', '#111');
      path.setAttribute('stroke', '#0a0a0a');
      path.setAttribute('stroke-width', '0.01');
      ringSvg.appendChild(path);
      ringArcs.push(path);
    }
  }
}

// rosbridge base64-encodes uint8[] fields — decode to Uint8Array if needed
function decodeBytes(data) {
  if (typeof data === 'string') {
    const binaryStr = atob(data);
    const out = new Uint8Array(binaryStr.length);
    for (let i = 0; i < binaryStr.length; i++) out[i] = binaryStr.charCodeAt(i);
    return out;
  }
  return data;
}

function onLedPixels(msg) {
  const bytes = decodeBytes(msg.data);
  if (!bytes || bytes.length < (LED_STRIP_COUNT + LED_RING_COUNT) * 3) return;

  // Strip
  for (let i = 0; i < LED_STRIP_COUNT; i++) {
    const r = bytes[i*3], g = bytes[i*3+1], b = bytes[i*3+2];
    if (stripRects[i]) stripRects[i].setAttribute('fill', `rgb(${r},${g},${b})`);
  }

  // Ring
  const off = LED_STRIP_COUNT * 3;
  for (let i = 0; i < LED_RING_COUNT; i++) {
    const r = bytes[off + i*3], g = bytes[off + i*3+1], b = bytes[off + i*3+2];
    if (ringArcs[i]) ringArcs[i].setAttribute('fill', `rgb(${r},${g},${b})`);
  }
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
  initLEDVisualizer();
  if (typeof THREE !== 'undefined') {
    initScene();
    animate3D();
  } else {
    log('Three.js not loaded — 3D view unavailable');
  }
});
