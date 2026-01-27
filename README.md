# ROS 2 WASD Teleoperation Node (Deadman Safety)

This package provides a **custom ROS 2 teleoperation node** for mobile robots (e.g. TurtleBot3) using **WASD-style keyboard control** with **deadman safety behavior**.

Unlike the default `teleop_twist_keyboard`, this node is designed to behave more like a game controller:
- Motion only occurs **while keys are actively held**
- Releasing keys causes the robot to **stop automatically**
- Speed limits are enforced centrally in the control loop

The goal is to provide **predictable, safe, and reusable human-in-the-loop control** suitable for demos, experiments, and lab use.

---

## Features

- WASD keyboard control
- Deadman-style safety (release key → stop)
- Independent forward and turning control
- Emergency stop (`Space`)
- Clean shutdown (robot stops on exit)
- No external GUI dependencies (terminal-based)

---

## Controls

| Key | Action |
|----|-------|
| `w` | Move forward |
| `s` | Move backward |
| `a` | Turn left |
| `d` | Turn right |
| `Space` | Emergency stop |
| `q` | Quit teleop node |

> ⚠️ The terminal running the teleop node **must be focused** for key input to work.

---

## How It Works (Design Overview)

The node is structured using a **separation of concerns** approach:

- **Keyboard input thread**
  - Reads raw key presses
  - Updates internal motion state
- **ROS timer callback**
  - Runs at a fixed rate
  - Enforces deadman timeout
  - Clamps velocities to safe limits
  - Publishes `/cmd_vel`

This ensures:
- Smooth motion
- Predictable behavior
- No blocking of the ROS executor

Deadman behavior is implemented by tracking the **last time a motion key was observed**.  
If no key has been seen within a short timeout, the corresponding motion axis is set to zero.

---

## Installation

Clone this package into your ROS 2 workspace:

```bash
cd ~/turtlebot3_ws/src
git clone git@github.com:JonathanSoucie/ros2_wasd_teleop.git
Build the workspace:

cd ~/turtlebot3_ws
colcon build --symlink-install
Source the workspace:

source ~/turtlebot3_ws/install/setup.bash

Running with TurtleBot3 (Gazebo)
Terminal 1: Launch Gazebo
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

Terminal 2: Run teleop node
ros2 run my_robot_control teleop_wasd

(Replace my_robot_control and teleop_wasd if your package or executable names differ.)
```

## Limitations
This node uses terminal-based keyboard input, which has inherent limitations:

True key-up events are not available

Multiple simultaneous keys are approximated via timing

For full key-state tracking (true simultaneous input), OS-level keyboard libraries (e.g. pynput) would be required.

These limitations are documented intentionally to highlight system-level tradeoffs.

## Intended Use
Mobile robotics experiments

Safe teleoperation during development

Teaching ROS 2 node design

Human-in-the-loop control research

## Author
Jonathan Soucie
Computer Engineering – Robotics
University of Ottawa
