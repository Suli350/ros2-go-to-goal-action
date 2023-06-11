"""Command line action client.

    ros2 run go_to_goal goal_client 8.0 3.0            # position only
    ros2 run go_to_goal goal_client 8.0 3.0 1.57       # position + final heading
    ros2 run go_to_goal goal_client --square            # visit the corners of a square
"""
import argparse
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from go_to_goal_interfaces.action import GoToPose


class GoalClient(Node):

    def __init__(self):
        super().__init__('goal_client')
        self.client = ActionClient(self, GoToPose, 'go_to_pose')

    def send(self, x, y, theta=None):
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available. Is goal_server running?')
            return False
        goal = GoToPose.Goal(x=x, y=y, theta=theta or 0.0, use_final_heading=theta is not None)
        future = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal rejected')
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        res = result_future.result().result
        self.get_logger().info(
            f'success={res.success} dist_err={res.final_distance_error:.3f} '
            f'heading_err={res.final_heading_error:.3f} time={res.elapsed_time:.1f}s')
        return res.success

    def on_feedback(self, msg):
        fb = msg.feedback
        self.get_logger().info(
            f'[{fb.phase:>13}] remaining={fb.distance_remaining:.2f} m  '
            f'heading_err={fb.heading_error:+.2f} rad', throttle_duration_sec=0.5)


def main(args=None):
    parser = argparse.ArgumentParser(description='Send GoToPose goals')
    parser.add_argument('x', type=float, nargs='?')
    parser.add_argument('y', type=float, nargs='?')
    parser.add_argument('theta', type=float, nargs='?')
    parser.add_argument('--square', action='store_true', help='drive around a square')
    opts = parser.parse_args(remove_ros_args(sys.argv)[1:])
    if not opts.square and (opts.x is None or opts.y is None):
        parser.error('give x and y, or --square')

    rclpy.init(args=args)
    node = GoalClient()
    try:
        if opts.square:
            for x, y, th in [(3.0, 3.0, 0.0), (8.0, 3.0, 1.57), (8.0, 8.0, 3.14), (3.0, 8.0, -1.57)]:
                if not node.send(x, y, th):
                    break
        else:
            node.send(opts.x, opts.y, opts.theta)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
