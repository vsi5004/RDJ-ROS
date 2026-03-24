"""
led_controller_node.py — RDJ vinyl robot LED controller.

Subscribes to:
  /motion/status         (MotionStatus) — axis positions and move targets
  /safety/velocity_scale (Float32)      — safety scaling factor (0.0–1.0)
  /safety/estop          (Bool)         — hardware e-stop flag
  /led/pattern           (String)       — semantic pattern name from state machine

Publishes:
  /led/pixels  (UInt8MultiArray)  — per-LED RGB pixel data at ~20 Hz
                                    Layout: [R,G,B]×strip_leds + [R,G,B]×ring_leds

For real hardware, writes dominant color to CAN via SDO:
  a_axis node  object 0x2220 — DotStar ring (60 LEDs, 360°)
  player node  object 0x2310 — DotStar X-rail strip (144 LEDs)

Pattern priority (highest first):
  1. ESTOP          — solid red     (from /safety/estop)
  2. SAFETY_WARNING — orange flash  (velocity_scale < threshold)
  3. MOVING         — green fill-to-target chase (when x/a moving in OPERATIONAL)
  4. <semantic>     — pattern from /led/pattern (HOMING, PLAYING, FAULT, …)
  5. IDLE           — slow breathing white (default)
"""

import math

import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import String, UInt8MultiArray

from vinyl_robot_msgs.msg import MotionStatus, SafetyStatus

# ── Color table (R, G, B) ─────────────────────────────────────────────────────

COLORS = {
    "ESTOP": (255, 0, 0),
    "SAFETY_WARNING": (255, 80, 0),
    "HOMING": (0, 80, 255),
    "OPERATIONAL": (255, 255, 255),
    "PLAYING": (0, 200, 50),
    "FAULT": (255, 80, 0),
    "FAULT_PARKED": (200, 0, 0),
    "MOVING": (0, 200, 50),
    "IDLE": (200, 200, 200),
}


# ── LED controller node ───────────────────────────────────────────────────────


class LEDControllerNode(Node):
    def __init__(self):
        super().__init__("led_controller")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("config_path", "")
        config_path = self.get_parameter("config_path").get_parameter_value().string_value

        cfg = {}
        if config_path:
            with open(config_path) as f:
                full = yaml.safe_load(f)
            cfg = full.get("led_controller", {})

        self._mock_mode: bool = cfg.get("mock_mode", True)
        self._brightness: float = float(cfg.get("brightness", 0.8))
        anim_hz: float = float(cfg.get("anim_hz", 20.0))
        self._strip_leds: int = int(cfg.get("strip_leds", 144))
        self._ring_leds: int = int(cfg.get("ring_leds", 60))
        self._x_travel_mm: float = float(cfg.get("x_travel_mm", 600.0))
        self._a_travel_deg: float = float(cfg.get("a_travel_deg", 360.0))
        self._breathe_frames: int = max(1, int(cfg.get("breathe_period_s", 2.0) * anim_hz))
        self._pulse_frames: int = max(1, int(cfg.get("pulse_period_s", 1.0) * anim_hz))
        self._sweep_frames: int = max(1, int(cfg.get("sweep_period_s", 2.0) * anim_hz))
        self._flash_frames: int = max(1, int(cfg.get("flash_period_s", 0.5) * anim_hz))
        self._warn_threshold: float = float(cfg.get("safety_warning_threshold", 1.0))
        self._fade_speed: float = float(cfg.get("fade_in_speed", 0.15))

        # ── Priority input state ───────────────────────────────────────────────
        self._estop_active: bool = False
        self._velocity_scale: float = 1.0
        self._semantic_pattern: str = "IDLE"

        # ── Motion state (from MotionStatus) ──────────────────────────────────
        self._x_mm: float = 0.0
        self._x_target_mm: float = 0.0
        self._a_deg: float = 0.0
        self._a_target_deg: float = 0.0
        self._x_moving: bool = False
        self._a_moving: bool = False

        # ── Per-LED alpha buffers (0.0–1.0), persist across frames ────────────
        self._strip_alpha: list = [0.0] * self._strip_leds
        self._ring_alpha: list = [0.0] * self._ring_leds

        # ── Animation clock ────────────────────────────────────────────────────
        self._frame: int = 0
        self._prev_pattern: str = ""

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(MotionStatus, "/motion/status", self._on_status, 10)
        self.create_subscription(SafetyStatus, "/safety/status", self._on_safety_status, 10)
        self.create_subscription(String, "/led/pattern", self._on_pattern, 10)

        # ── Publisher ─────────────────────────────────────────────────────────
        self._pixels_pub = self.create_publisher(UInt8MultiArray, "/led/pixels", 10)

        # ── SDO clients (real hardware only) ──────────────────────────────────
        self._sdo_a = None
        self._sdo_player = None
        self._COData = None
        if not self._mock_mode:
            try:
                from canopen_interfaces.srv import COData  # type: ignore

                self._COData = COData
                self._sdo_a = self.create_client(COData, "/a_axis/sdo_write")
                self._sdo_player = self.create_client(COData, "/player/sdo_write")
            except ImportError:
                self.get_logger().warn(
                    "canopen_interfaces not available — LED SDO writes disabled"
                )

        # ── Animation timer ───────────────────────────────────────────────────
        self.create_timer(1.0 / anim_hz, self._tick)

        self.get_logger().info(
            f"led_controller ready  mock={self._mock_mode}  "
            f"strip={self._strip_leds}  ring={self._ring_leds}  hz={anim_hz}"
        )

    # ── Subscription callbacks ─────────────────────────────────────────────────

    def _on_status(self, msg: MotionStatus) -> None:
        self._x_mm = msg.x_mm
        self._x_target_mm = msg.x_target_mm
        self._a_deg = msg.a_deg
        self._a_target_deg = msg.a_target_deg
        self._x_moving = msg.x_moving
        self._a_moving = msg.a_moving

    def _on_safety_status(self, msg: SafetyStatus) -> None:
        self._velocity_scale = float(msg.velocity_scale)
        self._estop_active = bool(msg.estop)

    def _on_pattern(self, msg: String) -> None:
        self._semantic_pattern = msg.data

    # ── Pattern priority resolver ──────────────────────────────────────────────

    def _resolve_pattern(self) -> str:
        if self._estop_active:
            return "ESTOP"
        if self._velocity_scale < self._warn_threshold:
            return "SAFETY_WARNING"
        # Show fill-to-target animation for any operational motion (not during homing or fault)
        _no_move_override = {"HOMING", "FAULT", "FAULT_PARKED"}
        if (self._x_moving or self._a_moving) and self._semantic_pattern not in _no_move_override:
            return "MOVING"
        return self._semantic_pattern if self._semantic_pattern in COLORS else "IDLE"

    # ── Animation helpers ──────────────────────────────────────────────────────

    def _breathe(self, period_frames: int) -> float:
        t = (self._frame % period_frames) / period_frames
        return 0.15 + 0.85 * (0.5 - 0.5 * math.cos(2.0 * math.pi * t))

    def _pulse(self, period_frames: int) -> float:
        t = (self._frame % period_frames) / period_frames
        return 0.1 + 0.9 * (0.5 - 0.5 * math.cos(2.0 * math.pi * t))

    def _flash(self, period_frames: int) -> float:
        return 1.0 if (self._frame % period_frames) < (period_frames // 2) else 0.0

    # ── Fill-to-target alpha update ────────────────────────────────────────────

    def _update_axis_alpha(
        self,
        alpha: list,
        current: float,
        target: float,
        travel: float,
        n_leds: int,
    ) -> None:
        """Update per-LED brightness for fill-to-target chase animation."""
        cur_idx = int(max(0.0, min(1.0, current / travel)) * (n_leds - 1))
        tgt_idx = int(max(0.0, min(1.0, target / travel)) * (n_leds - 1))
        lo = min(cur_idx, tgt_idx)
        hi = max(cur_idx, tgt_idx)

        for i in range(n_leds):
            if lo <= i <= hi:
                if i <= cur_idx:
                    # Axis has passed — fade fully in
                    alpha[i] = min(1.0, alpha[i] + self._fade_speed)
                else:
                    # Ahead of position — dim ghost showing planned path
                    alpha[i] = 0.25
            else:
                # Outside fill range — fade out
                alpha[i] = max(0.0, alpha[i] - self._fade_speed * 0.5)

    # ── Pixel buffer builder ───────────────────────────────────────────────────

    def _build_pixels(self, pattern: str) -> bytes:
        """Return flat [R,G,B]×strip_leds + [R,G,B]×ring_leds bytes."""
        r, g, b = COLORS.get(pattern, COLORS["IDLE"])
        buf = bytearray(3 * (self._strip_leds + self._ring_leds))

        if pattern == "MOVING":
            self._update_axis_alpha(
                self._strip_alpha,
                self._x_mm,
                self._x_target_mm,
                self._x_travel_mm,
                self._strip_leds,
            )
            self._update_axis_alpha(
                self._ring_alpha,
                self._a_deg,
                self._a_target_deg,
                self._a_travel_deg,
                self._ring_leds,
            )
            for i in range(self._strip_leds):
                a = self._strip_alpha[i] * self._brightness
                buf[i * 3] = int(r * a)
                buf[i * 3 + 1] = int(g * a)
                buf[i * 3 + 2] = int(b * a)
            off = self._strip_leds * 3
            for i in range(self._ring_leds):
                a = self._ring_alpha[i] * self._brightness
                buf[off + i * 3] = int(r * a)
                buf[off + i * 3 + 1] = int(g * a)
                buf[off + i * 3 + 2] = int(b * a)

        elif pattern == "HOMING":
            # Single bright dot sweeping around ring, mirror sweep on strip
            self._strip_alpha = [0.0] * self._strip_leds
            self._ring_alpha = [0.0] * self._ring_leds
            sweep_ring = (self._frame % self._sweep_frames) * self._ring_leds // self._sweep_frames
            sweep_strip = (
                (self._frame % self._sweep_frames) * self._strip_leds // self._sweep_frames
            )
            for i in range(self._strip_leds):
                dist = abs(i - sweep_strip)
                a = max(0.0, 1.0 - dist / 5.0) * self._brightness
                buf[i * 3] = int(r * a)
                buf[i * 3 + 1] = int(g * a)
                buf[i * 3 + 2] = int(b * a)
            off = self._strip_leds * 3
            for i in range(self._ring_leds):
                dist = min(abs(i - sweep_ring), self._ring_leds - abs(i - sweep_ring))
                a = max(0.0, 1.0 - dist / 3.0) * self._brightness
                buf[off + i * 3] = int(r * a)
                buf[off + i * 3 + 1] = int(g * a)
                buf[off + i * 3 + 2] = int(b * a)

        else:
            # Uniform animations applied to all LEDs
            self._strip_alpha = [0.0] * self._strip_leds
            self._ring_alpha = [0.0] * self._ring_leds

            if pattern == "ESTOP":
                a = self._brightness
            elif pattern in ("SAFETY_WARNING", "FAULT"):
                a = self._flash(self._flash_frames) * self._brightness
            elif pattern in ("OPERATIONAL", "IDLE"):
                a = self._breathe(self._breathe_frames) * self._brightness
            elif pattern == "PLAYING":
                # Slow, low-amplitude breathe — calm glow while record plays
                a = (0.25 + 0.35 * self._breathe(self._breathe_frames)) * self._brightness
            elif pattern == "FAULT_PARKED":
                a = self._pulse(self._breathe_frames) * self._brightness
            else:
                a = self._breathe(self._breathe_frames) * self._brightness

            ri, gi, bi = int(r * a), int(g * a), int(b * a)
            for i in range(self._strip_leds):
                buf[i * 3] = ri
                buf[i * 3 + 1] = gi
                buf[i * 3 + 2] = bi
            off = self._strip_leds * 3
            for i in range(self._ring_leds):
                buf[off + i * 3] = ri
                buf[off + i * 3 + 1] = gi
                buf[off + i * 3 + 2] = bi

        return bytes(buf)

    # ── Dominant color helper for SDO ──────────────────────────────────────────

    @staticmethod
    def _dominant_color(pixels: bytes, n_leds: int, offset: int = 0) -> int:
        """Average lit LEDs into one UINT32 ARGB (0xAARRGGBB little-endian)."""
        tr = tg = tb = count = 0
        for i in range(n_leds):
            pr = pixels[offset + i * 3]
            pg = pixels[offset + i * 3 + 1]
            pb = pixels[offset + i * 3 + 2]
            if pr or pg or pb:
                tr += pr
                tg += pg
                tb += pb
                count += 1
        if count == 0:
            return 0
        mr, mg, mb = tr // count, tg // count, tb // count
        return (0xFF << 24) | (mb << 16) | (mg << 8) | mr

    # ── SDO write helper ──────────────────────────────────────────────────────

    def _send_led_sdo(self, client, obj_index: int, color_u32: int) -> None:
        if client is None or self._COData is None:
            return
        if not client.service_is_ready():
            return
        req = self._COData.Request()
        req.index = obj_index
        req.subindex = 0
        req.data = color_u32
        client.call_async(req)

    # ── Main animation tick ───────────────────────────────────────────────────

    def _tick(self) -> None:
        pattern = self._resolve_pattern()

        if pattern != self._prev_pattern:
            self._prev_pattern = pattern
            # Start breathe-based patterns at peak brightness so transitions
            # don't flash dark (frame counter could be at trough otherwise).
            # Sweep and fill-to-target patterns start from 0 for correct motion sync.
            if pattern not in ("HOMING", "MOVING"):
                self._frame = self._breathe_frames // 2

        pixels = self._build_pixels(pattern)

        # Publish per-LED pixel array (consumed by web UI via rosbridge)
        msg = UInt8MultiArray()
        msg.data = list(pixels)
        self._pixels_pub.publish(msg)

        # CAN SDO output
        strip_color = self._dominant_color(pixels, self._strip_leds, offset=0)
        ring_color = self._dominant_color(pixels, self._ring_leds, offset=self._strip_leds * 3)

        if self._mock_mode:
            self.get_logger().debug(
                f"led [{pattern}]  strip=0x{strip_color:08X}  ring=0x{ring_color:08X}"
            )
        else:
            self._send_led_sdo(self._sdo_player, 0x2310, strip_color)
            self._send_led_sdo(self._sdo_a, 0x2220, ring_color)

        self._frame += 1


# ── Entry point ───────────────────────────────────────────────────────────────


def main(args=None):
    rclpy.init(args=args)
    node = LEDControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
