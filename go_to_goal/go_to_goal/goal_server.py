"""Action server that drives turtle1 to a pose using two PID loops.

The motion is split into phases that are reported as feedback:
  align          -> rotate in place towards the goal
  drive          -> drive forward while correcting heading
  final_heading  -> rotate to the requested final heading (optional)

Concepts: action server, goal/cancel callbacks, feedback, callback groups
and the MultiThreadedExecutor (so pose updates keep arriving while the
long-running execute callback is busy).
"""
import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from turtlesim.msg import Pose

from go_to_goal.pid import PID, clamp, normalize_angle
from go_to_goal_interfaces.action import GoToPose

WORLD_MIN, WORLD_MAX = 0.0, 11.08


class GoalServer(Node):

    def __init__(self):
        super().__init__('goal_server')
        p = self.declare_parameter
        p('rate', 30.0)
        p('distance_tolerance', 0.05)
        p('heading_tolerance', 0.02)
        p('align_tolerance', 0.25)
        p('max_linear_speed', 2.0)
        p('max_angular_speed', 3.0)
        p('timeout', 60.0)
        p('linear_pid', [1.2, 0.0, 0.05])
        p('angular_pid', [5.0, 0.1, 0.2])

        self.group = ReentrantCallbackGroup()
        self.pose = None
        self.busy = threading.Lock()
        self.cmd_pub = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.create_subscription(Pose, 'turtle1/pose', self.on_pose, 10, callback_group=self.group)
        self.server = ActionServer(
            self, GoToPose, 'go_to_pose',
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.group)
        self.get_logger().info('go_to_pose action server ready')

    def param(self, name):
        return self.get_parameter(name).value

    def on_pose(self, msg):
        self.pose = msg

    def on_goal(self, goal):
        inside = all(WORLD_MIN + 0.2 <= v <= WORLD_MAX - 0.2 for v in (goal.x, goal.y))
        if not inside:
            self.get_logger().warn(f'Rejecting goal ({goal.x:.2f}, {goal.y:.2f}): outside the world')
            return GoalResponse.REJECT
        if self.busy.locked():
            self.get_logger().warn('Rejecting goal: another goal is active')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def stop(self):
        self.cmd_pub.publish(Twist())

    def execute(self, goal_handle):
        with self.busy:
            return self._execute(goal_handle)

    def _execute(self, goal_handle):
        goal = goal_handle.request
        self.get_logger().info(f'New goal: x={goal.x:.2f} y={goal.y:.2f} theta={goal.theta:.2f}')
        lin = PID(*self.param('linear_pid'), output_limit=self.param('max_linear_speed'), integral_limit=1.0)
        ang = PID(*self.param('angular_pid'), output_limit=self.param('max_angular_speed'), integral_limit=0.5)
        dt = 1.0 / self.param('rate')
        start = time.monotonic()
        phase = 'align'
        feedback = GoToPose.Feedback()
        dist, heading_err = float('inf'), 0.0

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.stop()
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return self.result(False, dist, heading_err, start)
            if time.monotonic() - start > self.param('timeout'):
                self.stop()
                goal_handle.abort()
                self.get_logger().warn('Goal aborted: timeout')
                return self.result(False, dist, heading_err, start)

            pose = self.pose
            if pose is None:
                time.sleep(dt)
                continue

            dx, dy = goal.x - pose.x, goal.y - pose.y
            dist = math.hypot(dx, dy)
            bearing_err = normalize_angle(math.atan2(dy, dx) - pose.theta)
            cmd = Twist()
            previous = phase

            if phase in ('align', 'drive') and dist < self.param('distance_tolerance'):
                phase = 'final_heading' if goal.use_final_heading else 'done'
            elif phase == 'align':
                heading_err = bearing_err
                if abs(bearing_err) < self.param('align_tolerance'):
                    phase = 'drive'
                else:
                    cmd.angular.z = ang.update(bearing_err, dt)
            elif phase == 'drive':
                heading_err = bearing_err
                if abs(bearing_err) > 1.0:
                    phase = 'align'  # we drifted too far, stop and re-align
                else:
                    cmd.linear.x = clamp(lin.update(dist, dt), 0.0, self.param('max_linear_speed'))
                    cmd.angular.z = ang.update(bearing_err, dt)
            elif phase == 'final_heading':
                heading_err = normalize_angle(goal.theta - pose.theta)
                if abs(heading_err) < self.param('heading_tolerance'):
                    phase = 'done'
                else:
                    cmd.angular.z = ang.update(heading_err, dt)

            if phase != previous:
                lin.reset()
                ang.reset()
                self.get_logger().info(f'phase: {previous} -> {phase}')

            if phase == 'done':
                self.stop()
                goal_handle.succeed()
                self.get_logger().info(f'Goal reached in {time.monotonic() - start:.1f}s')
                return self.result(True, dist, heading_err, start)

            self.cmd_pub.publish(cmd)
            feedback.distance_remaining = dist
            feedback.heading_error = heading_err
            feedback.phase = phase
            goal_handle.publish_feedback(feedback)
            time.sleep(dt)

        self.stop()
        goal_handle.abort()
        return self.result(False, dist, heading_err, start)

    @staticmethod
    def result(success, dist, heading_err, start):
        res = GoToPose.Result()
        res.success = success
        res.final_distance_error = float(dist) if math.isfinite(dist) else -1.0
        res.final_heading_error = float(heading_err)
        res.elapsed_time = time.monotonic() - start
        return res


def main(args=None):
    rclpy.init(args=args)
    node = GoalServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
