"""
state_machine_node.py — BehaviorTree orchestrator for the vinyl robot.

Wraps the py_trees BehaviourTree built by tree_builder.py and provides:
  - Blackboard initialisation and all topic subscription callbacks
  - 10 Hz tick timer
  - Direct LED publish on e-stop (faster than waiting for the next tick)
  - /user/command handler (re-home trigger)

The tree auto-homes on startup if ALL_HOMED is False.
"""

import yaml
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Bool, Float32, String, UInt8
import py_trees
import py_trees_ros.trees

from vinyl_robot_msgs.msg import MotionStatus

from . import blackboard_keys as K
from . import tree_builder


_BB_NAMESPACE = "/rdj"


class StateMachineNode(Node):

    def __init__(self):
        super().__init__("state_machine")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("config_path", "")
        self.declare_parameter("tick_rate_hz", 10.0)
        self.declare_parameter("progress_threshold", 0.92)

        config_path = (
            self.get_parameter("config_path").get_parameter_value().string_value
        )
        tick_hz = (
            self.get_parameter("tick_rate_hz").get_parameter_value().double_value
        )

        if not config_path:
            self.get_logger().fatal(
                "state_machine: config_path parameter is empty — cannot start"
            )
            raise RuntimeError("config_path not set")

        with open(config_path) as f:
            config = yaml.safe_load(f)

        # Override progress threshold from parameter (allows launch-time override)
        param_threshold = (
            self.get_parameter("progress_threshold").get_parameter_value().double_value
        )
        if "turntable_monitor" not in config:
            config["turntable_monitor"] = {}
        config["turntable_monitor"]["progress_threshold"] = param_threshold

        # ── Blackboard ────────────────────────────────────────────────────────
        py_trees.blackboard.Blackboard.enable_activity_stream(100)
        self._bb = py_trees.blackboard.Client(
            name="StateMachineNode", namespace=_BB_NAMESPACE
        )

        # Register all keys this node writes
        for key in [
            K.SAFETY_OK, K.ESTOP_ACTIVE, K.ALL_HOMED,
            K.MOTION_FAULT, K.MOTION_FAULT_MSG, K.PINCHER_TOF_MM,
            K.PLAYBACK_PROGRESS, K.PLAY_MODE,
            K.CURRENT_RECORD_IDX, K.PREV_RECORD_IDX,
            K.CURRENT_SIDE, K.QUEUE_SIZE, K.NEXT_ACTION,
            K.FORCE_REHOME,
        ]:
            self._bb.register_key(key, access=py_trees.common.Access.WRITE)

        # Initialise defaults
        queue_size = len(
            config.get("positions", {})
            .get("queue_stack", {})
            .get("slot_z_mm", [0])
        )
        self._bb.safety_ok          = True
        self._bb.estop_active       = False
        self._bb.all_homed          = False
        self._bb.motion_fault       = False
        self._bb.motion_fault_msg   = ""
        self._bb.pincher_tof_mm     = 9999.0  # assume clear until measured
        self._bb.playback_progress  = 0.0
        self._bb.play_mode          = "SEQUENTIAL"
        self._bb.current_record_index = 0
        self._bb.prev_record_index  = 0
        self._bb.current_side       = "A"
        self._bb.queue_size         = queue_size
        self._bb.next_action        = ""
        self._bb.force_rehome       = False

        # ── LED publisher (shared with tree behaviors) ─────────────────────
        self._led_pub = self.create_publisher(String, "/led/pattern", 10)

        # ── Subscriptions ──────────────────────────────────────────────────
        cb = ReentrantCallbackGroup()

        self.create_subscription(
            MotionStatus, "/motion/status", self._on_status, 10,
            callback_group=cb,
        )
        self.create_subscription(
            Float32, "/turntable/progress", self._on_progress, 10,
            callback_group=cb,
        )
        self.create_subscription(
            Float32, "/safety/velocity_scale", self._on_scale, 10,
            callback_group=cb,
        )
        self.create_subscription(
            Bool, "/safety/estop", self._on_hw_estop, 10,
            callback_group=cb,
        )
        self.create_subscription(
            String, "/user/command", self._on_user_cmd, 10,
            callback_group=cb,
        )
        self.create_subscription(
            String, "/user/play_mode", self._on_play_mode, 10,
            callback_group=cb,
        )
        self.create_subscription(
            UInt8, "/user/select_record", self._on_select_record, 10,
            callback_group=cb,
        )
        # Software e-stop from operator (web UI / button)
        self.create_subscription(
            Bool, "/user/estop", self._on_sw_estop, 10,
            callback_group=cb,
        )

        # ── Build and set up behaviour tree ───────────────────────────────
        root = tree_builder.build_tree(self, config)
        self._bt = py_trees_ros.trees.BehaviourTree(root=root)
        self._bt.setup(node=self, timeout=15.0)

        # ── Tick timer ────────────────────────────────────────────────────
        self._tick_count = 0
        self._last_tip = ""
        self._tip_start_tick = 0
        tick_period = 1.0 / max(tick_hz, 1.0)
        self.create_timer(tick_period, self._tick, callback_group=cb)

        self.get_logger().info(
            f"state_machine: behavior tree ready — ticking at {tick_hz:.0f} Hz, "
            f"queue_size={queue_size}, config={config_path}"
        )

    # ── Subscription callbacks (write to blackboard at full topic rate) ────────

    def _on_status(self, msg: MotionStatus) -> None:
        prev_homed = self._bb.all_homed
        prev_fault = self._bb.motion_fault

        self._bb.all_homed        = msg.all_homed
        self._bb.motion_fault     = msg.fault
        self._bb.motion_fault_msg = msg.fault_msg
        self._bb.pincher_tof_mm   = float(msg.pincher_tof_mm)
        self._bb.safety_ok = msg.velocity_scale > 0.0 and not msg.fault

        if msg.all_homed != prev_homed:
            self.get_logger().info(
                f"state_machine: all_homed changed → {msg.all_homed}"
            )
        if msg.fault != prev_fault:
            level = self.get_logger().warn if msg.fault else self.get_logger().info
            level(f"state_machine: motion_fault changed → {msg.fault}"
                  + (f" ({msg.fault_msg})" if msg.fault else ""))

    def _on_progress(self, msg: Float32) -> None:
        self._bb.playback_progress = float(msg.data)

    def _on_scale(self, msg: Float32) -> None:
        # velocity_scale alone doesn't fully determine safety_ok — _on_status
        # does the authoritative update.  This callback handles the case where
        # /motion/status arrives less frequently (shouldn't happen but defensive).
        if msg.data <= 0.0:
            self._bb.safety_ok = False

    def _on_hw_estop(self, msg: Bool) -> None:
        self._bb.estop_active = msg.data
        if msg.data:
            self.get_logger().warn("state_machine: hardware E-STOP asserted")
            self._publish_led("ESTOP")

    def _on_sw_estop(self, msg: Bool) -> None:
        self._bb.estop_active = msg.data
        if msg.data:
            self.get_logger().warn("state_machine: software E-STOP asserted")
            self._publish_led("ESTOP")

    def _on_play_mode(self, msg: String) -> None:
        valid = {"SEQUENTIAL", "SINGLE_REPEAT", "SIDE_REPEAT"}
        mode = msg.data.strip().upper()
        if mode in valid:
            self._bb.play_mode = mode
            self.get_logger().info(f"state_machine: play_mode → {mode}")
        else:
            self.get_logger().warn(
                f"state_machine: unknown play_mode '{msg.data}' — ignored"
            )

    def _on_select_record(self, msg: UInt8) -> None:
        idx = int(msg.data)
        size = self._bb.queue_size
        if 0 <= idx < size:
            self._bb.current_record_index = idx
            self.get_logger().info(f"state_machine: selected record index {idx}")
        else:
            self.get_logger().warn(
                f"state_machine: select_record index {idx} out of range [0, {size})"
            )

    def _on_user_cmd(self, msg: String) -> None:
        cmd = msg.data.strip().lower()
        self.get_logger().info(f"state_machine: user command '{cmd}'")
        if cmd == "home":
            self._bb.force_rehome = True
            self._bb.next_action  = ""  # reset any halt state
            self.get_logger().info("state_machine: re-home requested — force_rehome set")
        elif cmd == "estop":
            self._bb.estop_active = True
            self._publish_led("ESTOP")
        elif cmd == "estop_clear":
            self._bb.estop_active = False
            self.get_logger().info("state_machine: software e-stop cleared")

    # ── LED helper ─────────────────────────────────────────────────────────────

    def _publish_led(self, pattern: str) -> None:
        msg = String()
        msg.data = pattern
        self._led_pub.publish(msg)

    # ── Tick ───────────────────────────────────────────────────────────────────

    def _tick(self) -> None:
        self._bt.tick()

        # Log when the active behaviour changes.
        tip = self._bt.root.tip()
        tip_name = tip.name if tip is not None else "—"
        tip_msg  = getattr(tip, "feedback_message", "") if tip is not None else ""
        if tip_name != self._last_tip:
            self._last_tip      = tip_name
            self._tip_start_tick = self._tick_count
            self.get_logger().info(
                f"[tree] → {tip_name}"
                + (f": {tip_msg}" if tip_msg else "")
            )
        elif self._tick_count % 10 == 0 and tip_msg:
            # Repeat feedback every second while blocked on the same behaviour.
            elapsed = (self._tick_count - self._tip_start_tick) / 10.0
            self.get_logger().info(
                f"[tree] {tip_name} (+{elapsed:.0f}s): {tip_msg}"
            )

        self._tick_count += 1


def main(args=None) -> None:
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = StateMachineNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
