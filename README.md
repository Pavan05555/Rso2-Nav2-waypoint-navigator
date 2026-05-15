# ROS2 Nav2 Waypoint Navigator

Autonomous multi-goal navigation system built using ROS2 Jazzy, Nav2, and TurtleBot3 Burger in Gazebo simulation. This project implements a custom Python ROS2 node that sends sequential waypoint goals to the Nav2 stack for fully autonomous navigation.

---

# Demo


https://github.com/user-attachments/assets/3461fb5a-8949-49b6-ab54-e8ab15b7ea0d


---

# Features

- Autonomous waypoint navigation using Nav2 `BasicNavigator`
- Multi-goal mission planning with custom ROS2 Python node
- TurtleBot3 Burger simulation in Gazebo
- Localization and path planning using Nav2 + AMCL
- Real-time robot visualization in RViz2
- Sequential goal execution with feedback monitoring
- Customizable waypoint coordinates

---

# Tech Stack

| Tool | Purpose |
|---|---|
| ROS2 Jazzy | Robot middleware |
| Nav2 | Autonomous navigation framework |
| TurtleBot3 Burger | Mobile robot platform |
| Gazebo Classic | Robot simulation |
| RViz2 | Visualization |
| AMCL | Localization |
| Python | Waypoint navigation node |

---

# Prerequisites

- Ubuntu 24.04 / 22.04
- ROS2 Jazzy installed

Install required packages:

```bash
sudo apt install ros-jazzy-turtlebot3* \
                 ros-jazzy-turtlebot3-simulations \
                 ros-jazzy-navigation2 \
                 ros-jazzy-nav2-bringup
```

---

# Setup

## Clone the Repository

```bash
git clone <your-repository-link>
```

## Build the Workspace

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Set TurtleBot3 Model

```bash
export TURTLEBOT3_MODEL=burger
```

To make it permanent:

```bash
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
source ~/.bashrc
```

---

# Usage

## Terminal 1 — Launch Gazebo Simulation

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

---

## Terminal 2 — Launch Nav2 Stack

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
use_sim_time:=True \
map:=/opt/ros/jazzy/share/turtlebot3_navigation2/map/map.yaml
```

This launches:

- Map Server
- AMCL Localization
- Planner Server
- Controller Server
- BT Navigator
- Recovery Behaviors

---

## Terminal 3 — Run Waypoint Navigator

```bash
ros2 run my_navigator goal_sender
```

The robot will autonomously navigate through all predefined waypoints.

---

# How It Works

The custom `goal_sender.py` node uses Nav2’s `BasicNavigator` API to send waypoint goals sequentially to the Nav2 stack.

Nav2 internally handles:

- Global path planning
- Localization
- Obstacle avoidance
- Velocity command generation

```text
goal_sender.py
       │
       ▼
PoseStamped Goals
       │
       ▼
   Nav2 Stack
   ├── AMCL
   ├── Planner Server
   ├── Controller Server
   └── BT Navigator
           │
           ▼
        /cmd_vel
           │
           ▼
   TurtleBot3 Burger
```

---

# TF Tree

```text
map
 └── odom
      └── base_footprint
            └── base_link
```

- `map → odom` is published by AMCL
- `odom → base_link` is published from robot odometry

---

# Custom Waypoints

Modify waypoints inside:

```bash
my_navigator/my_navigator/goal_sender.py
```

Example:

```python
waypoints = [
    (1.0, 0.5),
    (-1.0, 1.0),
    (0.0, 0.0)
]
```

---

# Project Structure

```text
ros2_ws/
└── src/
    └── my_navigator/
        ├── my_navigator/
        │   └── goal_sender.py
        ├── package.xml
        ├── setup.py
        └── resource/
```

---

# Topics Used

| Topic | Purpose |
|---|---|
| `/scan` | LiDAR data |
| `/odom` | Robot odometry |
| `/cmd_vel` | Velocity commands |
| `/tf` | Coordinate transforms |
| `/map` | Occupancy grid map |

---

# Future Improvements

- Dynamic obstacle avoidance
- Interactive RViz goal selection
- Patrol mode
- Frontier exploration
- Multi-robot navigation
- SLAM-based autonomous mapping

---

# Author

## Pavan Kumar Pavada

- Electronics and Communication Engineering
- ROS2 | Robotics | Embedded Systems | AI
