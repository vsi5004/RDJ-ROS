# HMI, Web Interface, and Record Identification

## Design Principle

All human interface paths — web UI, onboard display, voice control — publish to the same ROS 2 topics. The robot's operational logic (behavior tree, motion coordinator) is fully decoupled from the input source. A command to flip a record is identical whether it came from a phone browser, a voice command, or the onboard display.

This is achieved via **rosbridge_websocket**, which bridges ROS 2 topics to WebSocket connections. Any web client can subscribe to status topics and publish commands without needing ROS 2 installed.

## Web Interface (Phone-Accessible)

A lightweight web UI served from the Toradex, accessible from any device on the local network (phone, tablet, laptop).

### Architecture

```
Phone/Tablet Browser
    ↕ WebSocket (ws://robot-ip:9090)
rosbridge_websocket node (ROS 2)
    ↕ ROS 2 topics
Behavior tree / Motion coordinator
```

### Features

- **Playback status**: Current record, side (A/B), playback progress bar
- **Queue management**: View record stack, reorder, select next record
- **Manual controls**: Play, pause, flip, skip to next record, return to queue
- **Mode selection**: Sequential playback, single record repeat, single side repeat
- **System status**: Axis positions, homing state, safety zone status, faults
- **Record metadata**: Album art, artist, track listing (from identification system)

### Implementation

- **Frontend**: Single-page app (React or vanilla JS), served by a lightweight HTTP server (nginx or Python) in the Docker container
- **WebSocket bridge**: `rosbridge_websocket` from the `rosbridge_suite` package
- **ROS 2 package**: Add `ros-humble-rosbridge-suite` to the Dockerfile

### Topics the Web UI Subscribes To

| Topic | Type | Content |
|-------|------|---------|
| `/motion/status` | `MotionStatus` | All axis positions, homed state, faults |
| `/turntable/progress` | `Float32` | Playback progress 0.0–1.0 |
| `/safety/velocity_scale` | `Float32` | Current safety scaling |
| `/record/metadata` | `AlbumMetadata` | Current record identification |
| `/diagnostics` | `DiagnosticArray` | System health |

### Topics the Web UI Publishes To

| Topic | Type | Content |
|-------|------|---------|
| `/user/command` | `String` | "play", "pause", "flip", "skip", "home" |
| `/user/play_mode` | `String` | "sequential", "repeat_record", "repeat_side" |
| `/user/select_record` | `UInt8` | Queue slot index to play next |

The behavior tree checks these user command topics and integrates them into its decision flow.

## Onboard Display (Optional)

A small display (3.5"–5" HDMI or DSI) mounted on the robot base showing:

- Current record and side
- Playback progress
- System status (homed, faults, safety)
- IP address for web UI access

Can run as a separate lightweight container on Torizon using Weston/Wayland, subscribing to the same ROS 2 topics as the web UI via rosbridge or native ROS 2.

## Voice Control (Optional, Future)

Voice input as another command source. The voice processing runs either:
- **On-device**: Wake word detection + local command recognition (limited vocabulary: "play", "flip", "next", "stop")
- **Cloud-based**: Stream audio to a speech-to-text API, parse intent, publish to `/user/command`

Either way, the output is a message on `/user/command` — the behavior tree doesn't know or care that it came from voice.

## Record Identification

Two complementary approaches for identifying which record is playing:

### Visual Label Recognition (Camera-Based)

The existing camera (used for tonearm tracking) can also capture the record label when the arm is positioned over the platter. This happens during the record handling workflow — the camera already has a view of the platter area.

**Pipeline:**
1. Capture label image when record is placed on platter (before starting playback)
2. Run OCR or vision AI on the label image to extract text (artist, album, catalog number)
3. Query Discogs API (preferred) or MusicBrainz API with extracted text
4. Discogs catalog number is the most reliable identifier — it's unique per pressing
5. Cache results locally so repeated plays of the same record don't re-query

**Discogs API:**
- Free tier for personal use (60 requests/minute)
- Search by catalog number, artist, or album title
- Returns full metadata: tracklist, year, genre, cover art URL
- Requires a user agent string and optional OAuth for higher rate limits

**Fallback for worn/stylized labels:**
A vision-capable AI API (Claude, GPT-4V) can identify records from label photos more robustly than pure OCR, handling stylized fonts, logos, and partial text. This would be an API call from the SBC, not on-device inference.

### Audio Fingerprinting

Capture a short audio sample from the turntable's audio output and match it against a fingerprint database.

**Services:**
- **AudD** — REST API, 10-second audio clips, returns track/artist/album
- **ACRCloud** — Similar service, widely used in production

This identifies the specific track playing, not just the album. Useful for displaying "now playing" information on the web UI.

**Integration**: A ROS 2 node captures audio from an ALSA device (the turntable's audio output routed through a USB audio interface or the SoM's audio input), sends a 10-second clip to the fingerprint API, and publishes the result.

### AlbumMetadata Message

```
# msg/AlbumMetadata.msg
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

### Node: record_identifier

Subscribes to `/camera/image_raw` and optionally an audio topic. Publishes to `/record/metadata`. Runs identification on record placement (visual) and optionally during playback (audio fingerprint).

## Additional ROS 2 Packages for HMI

Add to Dockerfile:

```dockerfile
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rosbridge-suite \
    && rm -rf /var/lib/apt/lists/*
```

The web frontend files (HTML/JS/CSS) are served by a simple HTTP server and bundled in the Docker image or mounted as a volume.
