"""
state_machine_node.py — BehaviorTree.CPP orchestrator (stub).

Full implementation to follow. This stub:
  - Subscribes to all required topics so they appear in ros2 topic list
  - Waits for all_homed before entering operational loop
  - Handles 'home' user command by calling the HomeAll action server
  - Logs current state for debugging

Full behavior tree (ReactiveSequence root):
  SafetyCheck → WaitForRecordFinished → MoveToPlayer → LiftRecord
  → DecideAction (flip or swap) → PlaceRecord → PressPlay → SetSpeed
  → WaitForSide → (loop)

See docs/BEHAVIOR_TREE.md for full specification.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Float32, Bool, String, UInt8
from vinyl_robot_msgs.msg import MotionStatus
from vinyl_robot_msgs.action import HomeAll


class StateMachineNode(Node):

    def __init__(self):
        super().__init__('state_machine')

        self._all_homed = False
        self._safety_ok = True
        self._progress = 0.0
        self._homing_in_progress = False

        cb = ReentrantCallbackGroup()

        # Subscriptions — all topics the full BT will need
        self.create_subscription(MotionStatus, '/motion/status', self._on_status, 10,
                                 callback_group=cb)
        self.create_subscription(Float32, '/turntable/progress', self._on_progress, 10,
                                 callback_group=cb)
        self.create_subscription(Float32, '/safety/velocity_scale', self._on_scale, 10,
                                 callback_group=cb)
        self.create_subscription(Bool, '/safety/estop', self._on_estop, 10,
                                 callback_group=cb)
        self.create_subscription(String, '/user/command', self._on_user_cmd, 10,
                                 callback_group=cb)
        self.create_subscription(String, '/user/play_mode', self._on_play_mode, 10,
                                 callback_group=cb)
        self.create_subscription(UInt8, '/user/select_record', self._on_select_record, 10,
                                 callback_group=cb)

        # Action client — HomeAll
        self._home_client = ActionClient(self, HomeAll, '/motion/home_all',
                                         callback_group=cb)

        self.create_timer(1.0, self._tick, callback_group=cb)
        self.get_logger().info('state_machine: stub running — waiting for home_all')

    def _on_status(self, msg: MotionStatus) -> None:
        self._all_homed = msg.all_homed

    def _on_progress(self, msg: Float32) -> None:
        self._progress = msg.data

    def _on_scale(self, msg: Float32) -> None:
        self._safety_ok = msg.data > 0.0

    def _on_estop(self, msg: Bool) -> None:
        if msg.data:
            self.get_logger().warn('state_machine: E-STOP — halting tree')

    def _on_user_cmd(self, msg: String) -> None:
        cmd = msg.data
        self.get_logger().info(f'state_machine: user command: {cmd}')
        if cmd == 'home':
            self._send_home_goal()

    def _on_play_mode(self, msg: String) -> None:
        self.get_logger().info(f'state_machine: play mode: {msg.data}')

    def _on_select_record(self, msg: UInt8) -> None:
        self.get_logger().info(f'state_machine: select record: {msg.data}')

    def _send_home_goal(self) -> None:
        if self._homing_in_progress:
            self.get_logger().warn('state_machine: homing already in progress')
            return
        if not self._home_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('state_machine: home_all action server not available')
            return
        self._homing_in_progress = True
        self.get_logger().info('state_machine: sending home_all goal')
        future = self._home_client.send_goal_async(
            HomeAll.Goal(),
            feedback_callback=self._home_feedback,
        )
        future.add_done_callback(self._home_goal_response)

    def _home_feedback(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        self.get_logger().info(f'state_machine: homing — {fb.current_phase}')

    def _home_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('state_machine: home_all goal rejected')
            self._homing_in_progress = False
            return
        self.get_logger().info('state_machine: home_all goal accepted')
        goal_handle.get_result_async().add_done_callback(self._home_result)

    def _home_result(self, future) -> None:
        self._homing_in_progress = False
        result = future.result().result
        if result.success:
            self.get_logger().info('state_machine: homing complete')
        else:
            self.get_logger().error(f'state_machine: homing failed — {result.error_msg}')

    def _tick(self) -> None:
        if not self._all_homed:
            self.get_logger().info('state_machine: waiting for all_homed …')
        else:
            self.get_logger().info(
                f'state_machine: homed, safety={self._safety_ok}, '
                f'progress={self._progress:.2f} — BT not yet implemented'
            )


def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
