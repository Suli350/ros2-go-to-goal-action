# ROS2 Go-To-Goal Action

Drive `turtlesim` to any pose with a **custom ROS2 action** and two **PID controllers**.

![stack](https://img.shields.io/badge/ROS2-Humble-blue) ![python](https://img.shields.io/badge/python-3.10-green)

## Packages

| Package | Type | Contents |
|---|---|---|
| `go_to_goal_interfaces` | ament_cmake | `action/GoToPose.action` |
| `go_to_goal` | ament_python | PID class, action server, CLI action client |

## What you learn

- Defining your own interface (`.action`) and generating code with `rosidl`
- Action servers: accept/reject goals, publish feedback, handle cancel, return results
- Why long-running callbacks need a `MultiThreadedExecutor` + `ReentrantCallbackGroup`
- PID control with anti-windup and output limits, tuned from a YAML file
- A small state machine (`align -> drive -> final_heading -> done`)

## Build & run

```bash
cd ~/ros2_ws/src && git clone https://github.com/<you>/ros2-go-to-goal-action.git
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash

ros2 launch go_to_goal go_to_goal.launch.py
# second terminal
ros2 run go_to_goal goal_client 8.0 3.0 1.57
ros2 run go_to_goal goal_client --square
```

Or talk to the action directly:

```bash
ros2 action list -t
ros2 action send_goal --feedback /go_to_pose go_to_goal_interfaces/action/GoToPose \
  "{x: 2.0, y: 9.0, theta: 0.0, use_final_heading: true}"
```

Press `Ctrl+C` in the client while the turtle moves to see cancellation.

## Tuning

Edit `config/pid.yaml` (or `ros2 param set /goal_server angular_pid "[6.0, 0.0, 0.3]"`)
and watch `/turtle1/pose` in `rqt_plot` to see overshoot and settling time change.

## Exercises

1. Allow a new goal to **preempt** the running one instead of rejecting it.
2. Add a `max_time` field to the goal and abort if it is exceeded.
3. Port the server to C++ with `rclcpp_action`.
