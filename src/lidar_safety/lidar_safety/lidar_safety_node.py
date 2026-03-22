"""
lidar_safety_node.py — SICK TIM571 LiDAR safety zone monitor.

Subscribes to /scan (sensor_msgs/LaserScan from sick_scan_xd) and
/canopen/a_axis/tpdo1 (for current A axis angle).

Defines two dynamic zones that rotate with the arm:
  Warning zone (400mm radius): scales velocity to 25%
  Stop zone    (200mm radius): halts all motion (scale = 0.0)

In simulation (mock mode), this node publishes scale=1.0 and estop=False
indefinitely since there is no real LiDAR scan available.

TODO (when connecting real hardware):
  - Implement baseline subtraction to ignore static environment
  - Tune zone radii from robot_params.yaml
  - Implement arm-direction-aware zone rotation
"""

import math
import struct
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, UInt8MultiArray
from sensor_msgs.msg import LaserScan


class LidarSafetyNode(Node):

    def __init__(self):
        super().__init__('lidar_safety')

        self.declare_parameter('mock_mode', False)
        self.declare_parameter('warning_radius_mm', 400.0)
        self.declare_parameter('stop_radius_mm', 200.0)

        self._mock = self.get_parameter('mock_mode').get_parameter_value().bool_value
        self._warn_r = self.get_parameter('warning_radius_mm').get_parameter_value().double_value / 1000.0
        self._stop_r = self.get_parameter('stop_radius_mm').get_parameter_value().double_value / 1000.0

        self._a_angle_deg: float = 0.0

        # Publishers
        self._scale_pub = self.create_publisher(Float32, '/safety/velocity_scale', 10)
        self._estop_pub = self.create_publisher(Bool, '/safety/estop', 10)

        if self._mock:
            # In mock mode: always safe
            self.create_timer(0.1, self._publish_safe)
            self.get_logger().info('lidar_safety: MOCK MODE — always publishing scale=1.0')
        else:
            # Real mode: subscribe to LiDAR scan
            self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
            self.create_subscription(
                UInt8MultiArray, '/canopen/a_axis/tpdo1', self._on_a_tpdo, 10
            )
            self.get_logger().info('lidar_safety: waiting for /scan from SICK TIM571')

    def _publish_safe(self) -> None:
        msg = Float32()
        msg.data = 1.0
        self._scale_pub.publish(msg)

        estop = Bool()
        estop.data = False
        self._estop_pub.publish(estop)

    def _on_a_tpdo(self, msg: UInt8MultiArray) -> None:
        data = bytes(msg.data)
        if len(data) >= 8:
            # Bytes 6-7 are pot angle (INT16, 0.1° units) for A axis
            raw = struct.unpack_from('<h', data, 6)[0]
            self._a_angle_deg = raw / 10.0

    def _on_scan(self, msg: LaserScan) -> None:
        # Determine minimum range in forward hemisphere (relative to arm direction)
        min_range = float('inf')
        num = len(msg.ranges)
        for i, r in enumerate(msg.ranges):
            if math.isnan(r) or r < msg.range_min or r > msg.range_max:
                continue
            min_range = min(min_range, r)

        scale_msg = Float32()
        estop_msg = Bool()

        if min_range <= self._stop_r:
            scale_msg.data = 0.0
            estop_msg.data = True
            self.get_logger().warn(f'lidar_safety: STOP zone (min_range={min_range:.2f}m)')
        elif min_range <= self._warn_r:
            scale_msg.data = 0.25
            estop_msg.data = False
        else:
            scale_msg.data = 1.0
            estop_msg.data = False

        self._scale_pub.publish(scale_msg)
        self._estop_pub.publish(estop_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarSafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
