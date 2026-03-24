"""
bridge_node.py — ROS 2 bridge: ros2_canopen ProxyDriver ↔ UInt8MultiArray PDOs.

ProxyDriver interface (one COData message per OD entry per SYNC):
  Subscribes: <node>/rpd  canopen_interfaces/msg/COData  (node → master, i.e. TPDO data)
  Publishes:  <node>/tpd  canopen_interfaces/msg/COData  (master → node, i.e. RPDO data)

motion_coordinator interface (packed byte arrays):
  Publishes:   /canopen/<node>/tpdo1  std_msgs/UInt8MultiArray
  Subscribes:  /canopen/<node>/rpdo1  std_msgs/UInt8MultiArray

The bridge accumulates incoming COData entries per node into a dict keyed by
(OD index, subindex), then assembles and publishes packed TPDOs at 50 Hz on a
timer matching the SYNC period.  Incoming RPDOs are immediately disassembled
and each OD entry is forwarded as a COData message on <node>/tpd.
"""

from canopen_interfaces.msg import COData
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

from canopen_bridge.codec import (
    assemble_a_axis_tpdo,
    assemble_servo_tpdo,
    assemble_stepper_tpdo,
    disassemble_servo_rpdo,
    disassemble_stepper_rpdo,
)

# 50 Hz SYNC period — matches the CANopen bus SYNC interval
_SYNC_DT = 0.02

_STEPPER_NODES = ("x_axis", "z_axis", "a_axis")
_SERVO_NODES = ("pincher", "player")
_ALL_NODES = _STEPPER_NODES + _SERVO_NODES


class CanopenBridgeNode(Node):
    def __init__(self):
        super().__init__("canopen_bridge")

        # Per-node COData accumulators: {node_name: {(index, subindex): data}}
        self._acc: dict[str, dict[tuple, int]] = {name: {} for name in _ALL_NODES}

        # ── Publishers: packed TPDO → motion_coordinator ──────────────────────
        self._tpdo_pubs: dict[str, rclpy.publisher.Publisher] = {}
        for name in _ALL_NODES:
            self._tpdo_pubs[name] = self.create_publisher(
                UInt8MultiArray, f"/canopen/{name}/tpdo1", 10
            )

        # ── Publishers: COData entries → ProxyDriver (RPDO direction) ─────────
        self._tpd_pubs: dict[str, rclpy.publisher.Publisher] = {}
        for name in _ALL_NODES:
            self._tpd_pubs[name] = self.create_publisher(
                COData, f"{name}/tpd", 10
            )

        # ── Subscribers: COData from ProxyDriver (TPDO direction) ─────────────
        for name in _ALL_NODES:
            self.create_subscription(
                COData,
                f"{name}/rpd",
                lambda msg, n=name: self._on_codata(n, msg),
                10,
            )

        # ── Subscribers: packed RPDO from motion_coordinator ──────────────────
        for name in _STEPPER_NODES:
            self.create_subscription(
                UInt8MultiArray,
                f"/canopen/{name}/rpdo1",
                lambda msg, n=name: self._on_stepper_rpdo(n, msg),
                10,
            )
        for name in _SERVO_NODES:
            self.create_subscription(
                UInt8MultiArray,
                f"/canopen/{name}/rpdo1",
                lambda msg, n=name: self._on_servo_rpdo(n, msg),
                10,
            )

        # ── 50 Hz timer: assemble + publish TPDOs ─────────────────────────────
        self.create_timer(_SYNC_DT, self._publish_tpdos)

        self.get_logger().info("canopen_bridge running (50 Hz)")

    def _on_codata(self, node_name: str, msg: COData) -> None:
        """Accumulate a single OD entry from ProxyDriver into the per-node dict."""
        self._acc[node_name][(msg.index, msg.subindex)] = msg.data

    def _publish_tpdos(self) -> None:
        """Assemble and publish packed TPDOs for all nodes from current accumulators."""
        for name in _STEPPER_NODES:
            acc = self._acc[name]
            raw = assemble_a_axis_tpdo(acc) if name == "a_axis" else assemble_stepper_tpdo(acc)
            msg = UInt8MultiArray()
            msg.data = list(raw)
            self._tpdo_pubs[name].publish(msg)

        for name in _SERVO_NODES:
            acc = self._acc[name]
            raw = assemble_servo_tpdo(acc)
            msg = UInt8MultiArray()
            msg.data = list(raw)
            self._tpdo_pubs[name].publish(msg)

    def _on_stepper_rpdo(self, node_name: str, msg: UInt8MultiArray) -> None:
        """Disassemble a packed stepper RPDO and forward each OD entry to ProxyDriver."""
        raw = bytes(msg.data)
        if len(raw) < 8:
            self.get_logger().warning(
                f"[{node_name}] stepper RPDO too short: {len(raw)} bytes"
            )
            return
        pub = self._tpd_pubs[node_name]
        for entry in disassemble_stepper_rpdo(raw):
            co = COData()
            co.index = entry["index"]
            co.subindex = entry["subindex"]
            co.data = entry["data"]
            pub.publish(co)

    def _on_servo_rpdo(self, node_name: str, msg: UInt8MultiArray) -> None:
        """Disassemble a packed servo RPDO and forward each OD entry to ProxyDriver."""
        raw = bytes(msg.data)
        if len(raw) < 5:
            self.get_logger().warning(
                f"[{node_name}] servo RPDO too short: {len(raw)} bytes"
            )
            return
        pub = self._tpd_pubs[node_name]
        for entry in disassemble_servo_rpdo(raw):
            co = COData()
            co.index = entry["index"]
            co.subindex = entry["subindex"]
            co.data = entry["data"]
            pub.publish(co)


def main(args=None):
    rclpy.init(args=args)
    node = CanopenBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
