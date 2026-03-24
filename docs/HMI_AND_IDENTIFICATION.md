# HMI, Web Interface, and Record Identification

## Design Principle

All HMI paths (web UI, onboard display, voice) publish to the same ROS 2 topics. Robot logic is fully decoupled from input source — the behavior tree doesn't know or care whether a command came from the web, a button, or voice. rosbridge_websocket bridges the browser to ROS 2.

## Web Interface (Implemented)

### Architecture

```
Browser ↔ WebSocket (ws://robot-ip:9090) ↔ rosbridge_websocket ↔ ROS 2 topics
Browser ← HTTP (http://robot-ip:8080) ← Python http.server ← web/ static files
```

roslibjs (`roslib.min.js`) is bundled in the Docker image at build time — no CDN dependency, works offline.

**Important**: `ROSLIB.ActionClient` (roslibjs) uses ROS 1 message naming conventions and does not work with ROS 2 rosbridge. All web → robot commands use plain topic publishes, not action calls. The state_machine bridges `/user/command` to action servers internally.

### UI Panels

**Status** — Live telemetry from `/motion/status` and `/safety/status`:
- Axis positions (X mm, Z mm, A °)
- Per-axis homed flags (X✓ Z✓ A✓)
- Safety velocity scale with colour coding (green/amber/red)
- ToF sensor readings (X axis, Z axis, pincher)
- Moving indicators (per axis)
- E-stop indicator (hardware estop from LiDAR, via `SafetyStatus.estop`)
- Fault banner (red, shows fault_msg when fault=true)

**Playback** — `/turntable/progress` as a progress bar (0–100%)

**Manual Jog** — Direct axis position control for hardware calibration and debugging:
- X, Z: ±1 / ±10 / ±50 / ±100 mm buttons
- A: ±1 / ±10 / ±45 / ±90 ° buttons
- Live position display per axis
- Position snippet (`x_mm: / z_mm: / a_deg:`) for copy-paste into `robot_params.yaml`
- Buttons disabled when not homed or e-stop active
- Publishes `"<axis> <delta>"` to `/motion/jog`; motion_coordinator applies immediately

**Servo Raw Control** — Live pulse-width sliders for servo endpoint calibration:
- Pincher S1 (grip), Pincher S2 (flip), Player S1 (play), Player S2 (speed)
- Range: 500–2500 µs, step 10 µs; updates live on drag
- Labelled with corresponding `robot_params.yaml` key for each endpoint
- Publishes `"<node> <s1_us> <s2_us>"` to `/motion/servo_raw`

**Emergency Stop**:
- Red ⛔ E-STOP button — activates software estop via `/user/estop: true`
- Clear E-Stop button — publishes `/user/estop: false`; only clears software estop (hardware estop from LiDAR must clear on its own)
- `Escape` keyboard shortcut triggers estop

**Controls** — High-level commands and queue management:
- **Home All** — publishes `"home"` to `/user/command`
- **Start Playing** — publishes `"start"` to `/user/command`; shown only before the first disc is loaded; triggers the `InitialLoad` sequence
- **Flip Now** — publishes `"flip"` to `/user/command`; disabled when no record is on the player
- **Play Mode** — publishes to `/user/play_mode`: `SEQUENTIAL`, `SIDE_REPEAT`, or `SINGLE_REPEAT`
- **Queue Slot selector** — publishes slot index to `/user/select_record` (UInt8); highlights current/pending slots; shows ●/○ occupancy

### Topics

**Subscribed by browser:**
| Topic | Type | Used for |
|---|---|---|
| `/motion/status` | `vinyl_robot_msgs/MotionStatus` | All axis + sensor display |
| `/turntable/progress` | `std_msgs/Float32` | Progress bar |
| `/safety/status` | `vinyl_robot_msgs/SafetyStatus` | Hardware estop indicator + velocity scale |
| `/state_machine/status` | `vinyl_robot_msgs/StateMachineStatus` | Slot indicator, side, player occupancy |
| `/state_machine/tip` | `std_msgs/String` | Active behaviour display |
| `/led/pixels` | `std_msgs/UInt8MultiArray` | LED strip/ring visualizer |

**Published by browser:**
| Topic | Type | Purpose |
|---|---|---|
| `/user/command` | `std_msgs/String` | High-level commands: `"home"`, `"start"`, `"flip"` |
| `/user/estop` | `std_msgs/Bool` | Software e-stop activate/clear |
| `/user/play_mode` | `std_msgs/String` | `SEQUENTIAL`, `SIDE_REPEAT`, `SINGLE_REPEAT` |
| `/user/select_record` | `std_msgs/UInt8` | Queue override — slot index to load next |
| `/motion/jog` | `std_msgs/String` | Axis jog increments |
| `/motion/servo_raw` | `std_msgs/String` | Raw servo pulse widths for calibration |

## Servo Endpoint Calibration Workflow

The servo endpoints in `robot_params.yaml` (`grip.open_pulse_us`, `flip.servo_pulse_a_us`, etc.) need to be determined for each physical build. Use the web UI Servo Raw Control panel:

1. Home the robot
2. Open the web UI at `http://robot-ip:8080`
3. Drag each servo slider until the mechanism reaches the desired position
4. Note the µs value shown next to the slider
5. Update the corresponding key in `config/robot_params.yaml`
6. Restart the container (config is volume-mounted, no rebuild needed)

## Onboard Display (Future)

Optional 3.5"–5" HDMI/DSI display showing current record/side, progress, status, IP address. Separate lightweight Torizon container using Weston/Wayland. Publishes to same `/user/*` topics.

## Voice Control (Future)

On-device wake word + local command recognition, or cloud speech-to-text. Output publishes to `/user/command` — same interface as web UI.

## Record Identification (Future)

### Visual Label Recognition
1. Capture label image when record is placed (camera already has platter view)
2. OCR or vision AI extracts text (artist, album, catalog number)
3. Query Discogs API (preferred) or MusicBrainz
4. Discogs catalog number is most reliable identifier (unique per pressing)
5. Fallback: vision AI (Claude, GPT-4V) for worn/stylized labels

### Audio Fingerprinting
1. Capture 10-second audio sample from turntable output
2. AudD or ACRCloud REST API returns track/artist/album
3. Identifies the specific track playing, not just the album

### AlbumMetadata Message
```
string artist
string album_title
string catalog_number
uint16 year
string genre
string[] tracklist
string cover_art_url
string identification_method    # "visual", "audio_fingerprint", "manual"
float32 confidence              # 0.0–1.0
```

**record_identifier node** *(not yet implemented)*: Subscribes `/camera/image_raw` (+ optional audio), publishes `/record/metadata`.
