"""
turntable_monitor_node.py — Camera-based tonearm position tracker.

CV pipeline (1-2 Hz is sufficient):
  1. Receive /camera/image_raw
  2. Crop to platter region of interest
  3. Convert to HSV, threshold for tonearm colour
  4. Find largest contour, compute centroid
  5. Map centroid radius to progress float (0.0=outer, 1.0=run-out)
  6. Publish to /turntable/progress

In simulation (mock_mode=True), publishes a slowly incrementing progress
value so the behavior tree can be exercised without a real camera.

TODO (when connecting real hardware):
  - Tune HSV thresholds for tonearm colour in robot_params.yaml
  - Calibrate platter center pixel and inner/outer groove radii
  - Add auto-return detection (tonearm suddenly moves back to outer edge)
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class TurntableMonitorNode(Node):

    def __init__(self):
        super().__init__('turntable_monitor')

        self.declare_parameter('mock_mode', False)
        self._mock = self.get_parameter('mock_mode').get_parameter_value().bool_value

        self._progress: float = 0.0
        self._pub = self.create_publisher(Float32, '/turntable/progress', 10)

        if self._mock:
            # Simulate a record playing: progress increments at ~1% per second
            # A full side takes ~25 minutes; we simulate faster (1% per tick at 1Hz)
            self.create_timer(1.0, self._mock_tick)
            self.get_logger().info('turntable_monitor: MOCK MODE — simulating progress')
        else:
            try:
                from sensor_msgs.msg import Image
                from cv_bridge import CvBridge
                self._bridge = CvBridge()
                self.create_subscription(Image, '/camera/image_raw', self._on_image, 5)
                self.get_logger().info('turntable_monitor: waiting for /camera/image_raw')
            except ImportError:
                self.get_logger().error('cv_bridge not available — falling back to mock mode')
                self._mock = True
                self.create_timer(1.0, self._mock_tick)

    def _mock_tick(self) -> None:
        # Hold at 0.0 until manually advanced (state_machine drives the record swap)
        # Real simulation would increment; for infra testing keep it stable
        msg = Float32()
        msg.data = self._progress
        self._pub.publish(msg)

    def _on_image(self, msg) -> None:
        try:
            import cv2
            import numpy as np
            frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
            progress = self._process_frame(frame)
            out = Float32()
            out.data = float(progress)
            self._pub.publish(out)
        except Exception as exc:
            self.get_logger().debug(f'turntable_monitor frame error: {exc}')

    def _process_frame(self, frame) -> float:
        import cv2
        import numpy as np

        h, w = frame.shape[:2]
        # Assume platter is centered in frame
        cx, cy = w // 2, h // 2

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Default tonearm colour: warm orange/yellow — tune via parameters
        lower = np.array([20, 100, 100])
        upper = np.array([40, 255, 255])
        mask = cv2.inRange(hsv, lower, upper)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return self._progress   # return last known value

        largest = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M['m00'] < 10:
            return self._progress

        tx = int(M['m10'] / M['m00'])
        ty = int(M['m01'] / M['m00'])

        # Radial distance from platter center
        radius = math.sqrt((tx - cx) ** 2 + (ty - cy) ** 2)

        # Calibration constants (pixels) — these should come from YAML eventually
        outer_r = min(w, h) * 0.45   # outer groove
        inner_r = min(w, h) * 0.12   # run-out groove

        progress = 1.0 - max(0.0, min(1.0, (radius - inner_r) / (outer_r - inner_r)))
        self._progress = progress
        return progress


def main(args=None):
    rclpy.init(args=args)
    node = TurntableMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
