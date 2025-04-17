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

- **ROS2 Nodes**
  - `auto_nav`: Navigation logic, LIDAR processing, and obstacle avoidance.
  - `hardware_control`: GPIO control logic, subscribing to a trigger topic.
- **Topics**
  - `/scan`: LIDAR scan data (LaserScan)
  - `/odom`: Odometry data (Odometry)
  - `/map`: Occupancy grid (OccupancyGrid)
  - `/cmd_vel`: Motion command output (Twist)
  - `/trigger_hardware`: Custom topic to initiate hardware action (Bool)

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
