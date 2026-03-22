"""
diagnostics_node.py — System health aggregator (stub).

Subscribes to /motion/status, /safety/velocity_scale, /safety/estop.
Publishes /diagnostics (diagnostic_msgs/DiagnosticArray) at 1 Hz.

Full implementation: add per-node heartbeat monitoring, CAN bus health,
camera connectivity, LiDAR connectivity.
"""

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from vinyl_robot_msgs.msg import MotionStatus
from std_msgs.msg import Float32, Bool


class DiagnosticsNode(Node):

    def __init__(self):
        super().__init__('diagnostics_aggregator')

        self._motion_status = None
        self._safety_scale = 1.0
        self._estop = False

        self.create_subscription(MotionStatus, '/motion/status', self._on_status, 10)
        self.create_subscription(Float32, '/safety/velocity_scale', self._on_scale, 10)
        self.create_subscription(Bool, '/safety/estop', self._on_estop, 10)

        self._diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_timer(1.0, self._publish_diagnostics)

        self.get_logger().info('diagnostics_aggregator running')

    def _on_status(self, msg: MotionStatus) -> None:
        self._motion_status = msg

    def _on_scale(self, msg: Float32) -> None:
        self._safety_scale = msg.data

    def _on_estop(self, msg: Bool) -> None:
        self._estop = msg.data

    def _publish_diagnostics(self) -> None:
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()

        # Motion coordinator status
        motion_stat = DiagnosticStatus()
        motion_stat.name = 'motion_coordinator'
        motion_stat.hardware_id = 'vinyl_robot'
        if self._motion_status is None:
            motion_stat.level = DiagnosticStatus.WARN
            motion_stat.message = 'No status received yet'
        elif self._motion_status.fault:
            motion_stat.level = DiagnosticStatus.ERROR
            motion_stat.message = self._motion_status.fault_msg
        else:
            motion_stat.level = DiagnosticStatus.OK
            motion_stat.message = 'OK' if self._motion_status.all_homed else 'Not homed'
        if self._motion_status:
            motion_stat.values = [
                KeyValue(key='x_mm', value=f'{self._motion_status.x_mm:.1f}'),
                KeyValue(key='z_mm', value=f'{self._motion_status.z_mm:.1f}'),
                KeyValue(key='a_deg', value=f'{self._motion_status.a_deg:.1f}'),
                KeyValue(key='all_homed', value=str(self._motion_status.all_homed)),
            ]
        array.status.append(motion_stat)

        # Safety status
        safety_stat = DiagnosticStatus()
        safety_stat.name = 'lidar_safety'
        safety_stat.hardware_id = 'vinyl_robot'
        if self._estop:
            safety_stat.level = DiagnosticStatus.ERROR
            safety_stat.message = 'E-STOP active'
        elif self._safety_scale < 1.0:
            safety_stat.level = DiagnosticStatus.WARN
            safety_stat.message = f'Velocity scaled to {self._safety_scale:.0%}'
        else:
            safety_stat.level = DiagnosticStatus.OK
            safety_stat.message = 'OK'
        safety_stat.values = [
            KeyValue(key='velocity_scale', value=f'{self._safety_scale:.2f}'),
            KeyValue(key='estop', value=str(self._estop)),
        ]
        array.status.append(safety_stat)

        self._diag_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
