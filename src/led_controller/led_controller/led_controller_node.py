"""
led_controller_node.py — DotStar LED pattern controller (stub).

Maps axis positions and motion state to LED animations on:
  - A axis DotStar ring (20 LEDs) via /canopen/a_axis/rpdo1
  - X rail DotStar strip via /canopen/player/rpdo1 (0x2310 object)

Full implementation:
  - Idle: slow breathing white
  - Homing: blue sweep
  - Moving: green chase in direction of motion
  - Gripping: yellow pulse
  - Safety warning: orange flash
  - E-stop: red solid

TODO: Implement pattern generator and CAN LED object writes.
"""

import rclpy
from rclpy.node import Node
from vinyl_robot_msgs.msg import MotionStatus
from std_msgs.msg import Float32, Bool


class LEDControllerNode(Node):

    def __init__(self):
        super().__init__('led_controller')

        self.create_subscription(MotionStatus, '/motion/status', self._on_status, 10)
        self.create_subscription(Float32, '/safety/velocity_scale', self._on_scale, 10)
        self.create_subscription(Bool, '/safety/estop', self._on_estop, 10)

        self.get_logger().info('led_controller: stub running')

    def _on_status(self, msg: MotionStatus) -> None:
        pass   # TODO: compute LED pattern from axis positions

    def _on_scale(self, msg: Float32) -> None:
        pass   # TODO: flash orange when scale < 1.0

    def _on_estop(self, msg: Bool) -> None:
        pass   # TODO: solid red on e-stop


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


if __name__ == '__main__':
    main()
