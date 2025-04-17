# CDE2310 Autonomous TurtleBot3 Project

## Overview

This repository contains the implementation of a ROS2-based autonomous robot system developed for the CDE2310 AY24/25 course project. The system is designed to control a TurtleBot3 (Burger) equipped with a Raspberry Pi 4, capable of navigating a constrained environment, identifying infrared heat signatures, and activating a projectile launcher mechanism.

## Project Objectives

The primary mission is as follows:

- Navigate an enclosed maze, not larger than 5m x 5m, autonomously.
- Detect heat-emitting targets within the enclosed maze.
- Orient and launch three ping pong balls at vertical markers positioned 1.5 meters above detected hot targets.
- Ensure accurate launch timing with a fixed sequence (2s-4s-2s).
- Avoid contact between projectiles and room walls or ceilings.

## Quick Start Guide

This guide outlines the necessary steps to set up and run the full system across both the TurtleBot3 platform and a remote computer.

### 0. Prerequisites

Follow the official TurtleBot3 quick start documentation to install all necessary dependencies, including ROS 2 Humble, on both the TurtleBot and the remote computer:
[https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)

### 1. Workspace Preparation

- Clone or place the following workspaces accordingly:
  - `turtlebot3_ws` → on the TurtleBot (Raspberry Pi)
  - `remote_computer_ws` → on the remote computer

### 2. Build Workspaces

On both devices, navigate to the root of the respective workspaces and run:

```bash
colcon build
source install/setup.bash
```

### 3. Launch File Setup

- Copy the contents of the `turtlebot3_launch` directory into the `~/launch` directory on the TurtleBot.
- Copy the contents of the `remote_computer_launch` directory into the `~/launch` directory on the remote computer.

### 4. System Execution

**On the TurtleBot (Raspberry Pi):**

Open two terminal sessions and execute:

```bash
# Terminal 1
ros2 launch turtlebot3_bringup robot.launch.py

# Terminal 2
ros2 launch raspi_launch.py
```

**On the Remote Computer:**

Open two terminal sessions and execute:

```bash
# Terminal 1
ros2 launch turtlebot3_cartographer cartographer.launch.py

# Terminal 2
ros2 launch robot_fsm_launch.py
```

Ensure that both systems share the same ROS domain ID and that networking (e.g., multicast over Wi-Fi) is properly configured for cross-device ROS communication.


## Repository Structure

```
.
├── remote_computer_launch/     # Launch files for the remote computer (navigation, FSM)
├── remote_computer_ws/         # ROS2 workspace containing remote-side source code and packages
├── turtlebot3_launch/          # Launch files for the TurtleBot3 (hardware bring-up, GPIO control)
├── turtlebot3_ws/              # ROS2 workspace for the TurtleBot3 (Raspberry Pi) environment
├── docs/                       # Project documentation, design review slides, final report, etc.
├── README.md                   # Project overview and setup instructions
└── .gitattributes              # Git attributes configuration
```

> All source code and configurations are organized by execution context (remote computer vs. onboard TurtleBot). Documentation and final deliverables are stored in the `docs/` directory for clarity and submission readiness.


## Software Architecture

The system is built around a multi-node ROS 2 architecture distributed across two platforms:

- **TurtleBot3 (Raspberry Pi 4)** – handles low-level hardware actuation and sensing.
- **Remote Computer** – manages high-level behavior, navigation, and supervision logic.

### Core Packages

- `cde2310_interfaces/`  
  Custom message and service definitions:
  - `ActivateNode.srv` – used to activate or deactivate a node.
  - `NodeFinish.srv` – used to indicate whether a node has completed its task.
  - `Frontier.msg` – contains metadata of a single frontier (size, centroid).
  - `FrontierList.msg` – contains a list of `Frontier` messages.

---

### Topics

| Topic                        | Message Type                      | Publisher Node(s)        | Description |
|-----------------------------|-----------------------------------|---------------------------|-------------|
| `/cmd_vel`                  | `geometry_msgs/msg/Twist`         | Multiple                  | Movement commands for TurtleBot3. |
| `/scan`                     | `sensor_msgs/msg/LaserScan`       | TurtleBot3 (built-in)     | LIDAR data. |
| `/odom`                     | `nav_msgs/msg/Odometry`           | TurtleBot3 (built-in)     | Robot pose and orientation. |
| `/map`                      | `nav_msgs/msg/OccupancyGrid`      | SLAM/cartographer         | Map for path planning and frontier search. |
| `/pure_pursuit_path`        | `nav_msgs/msg/Path`               | `frontier_exploration`    | Path to follow based on frontier exploration. |
| `/heat_source_detected`     | `std_msgs/msg/Float32MultiArray`  | `amg_sensor_node`         | Thermal sensor data (8x8 grid). |
| `/robot_fire`               | `ActivateNode.srv`                | `firing` (remote) → `raspi_firing` | Fires 3 ping pong balls. |
| `/target_detected`          | `ActivateNode.srv`                | `thermal_target` → `supervisor` | Signals when a target is detected. |

---

### Services

| Service Name                 | Type                  | Server Node         | Purpose |
|-----------------------------|-----------------------|---------------------|---------|
| `activate_exploration`      | `ActivateNode`        | `exploration`       | Starts/stops frontier exploration. |
| `pure_pursuit_finish`       | `NodeFinish`          | `exploration`       | Indicates end of pure pursuit. |
| `activate_pure_pursuit`     | `ActivateNode`        | `pure_pursuit`      | Toggles path-following behavior. |
| `activate_alignment`        | `ActivateNode`        | `alignment`         | Starts/stops thermal-based alignment. |
| `alignment_finish`          | `NodeFinish`          | `alignment`         | Indicates completion of alignment. |
| `activate_firing`           | `ActivateNode`        | `firing`, `raspi_firing` | Starts firing logic remotely and physically. |
| `firing_finish`             | `NodeFinish`          | `firing`            | Firing complete signal. |

---

### Node Overview

#### Remote Computer Nodes

| Node               | Function |
|--------------------|----------|
| `supervisor.py`     | Central FSM managing mission phases: exploration → alignment → firing. |
| `exploration.py`    | High-level controller for starting/stopping exploration. |
| `frontier_exploration.py` | Generates navigation goals from frontier-based exploration. |
| `pure_pursuit.py`   | Executes path-following to reach frontier goals. |
| `alignment.py`      | Adjusts heading based on AMG8833 thermal sensor readings. |
| `thermal_target.py` | Detects new thermal targets using temperature thresholding and spatial memory. |
| `firing.py`         | Relays firing commands to the robot and receives firing status. |

#### TurtleBot3 Nodes (Raspberry Pi)

| Node               | Function |
|--------------------|----------|
| `raspi_firing.py`  | Drives servo and flywheels to launch ping pong balls. |
| `raspi_thermal.py` | Publishes thermal data from the AMG8833 sensor. |

---

### Requirements

- ROS2 Humble
- Python 3.x
- NumPy
- RPi.GPIO (on Raspberry Pi)
- cv2

## Hardware Setup

- Servo Motor: Connected to GPIO Pin 18
- Solenoid: Connected to GPIO Pin 23
- Ensure appropriate electrical safety practices are followed, including the use of flyback diodes for solenoid protection.

## Testing Strategy

- Software-in-the-loop testing with LIDAR and map data outputs.
- Standalone testing of GPIO activation.
- Full integration test within a mock maze environment replicating mission constraints.


## Acknowledgements

This repository was developed as part of the CDE2310 course requirements at the College of Design and Engineering, National University of Singapore.

Team Members:
- Irfan
- Charlie Hee
- Manya Gupta
- Toh Leong Chuan
