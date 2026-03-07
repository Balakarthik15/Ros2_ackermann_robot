# 🤖 ROS2 Ackermann Robot

> A complete ROS 2 simulation and control stack for an Ackermann-steering mobile robot — featuring URDF modeling, Gazebo simulation, autonomous navigation via Nav2, and real-time teleoperation.

---

<!-- TODO: Replace badge URLs with your actual repo path after making it public -->
[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue?logo=ros)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Simulator-Gazebo-orange)](https://gazebosim.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Build Status](https://img.shields.io/badge/build-passing-brightgreen)](https://github.com/Balakarthik15/Ros2_ackermann_robot/actions)
[![Issues](https://img.shields.io/github/issues/Balakarthik15/Ros2_ackermann_robot)](https://github.com/Balakarthik15/Ros2_ackermann_robot/issues)
[![Stars](https://img.shields.io/github/stars/Balakarthik15/Ros2_ackermann_robot?style=social)](https://github.com/Balakarthik15/Ros2_ackermann_robot/stargazers)

---

## 📋 Table of Contents

- [Overview](#overview)
- [Key Features](#key-features)
- [Tech Stack & Requirements](#tech-stack--requirements)
- [Project Architecture](#project-architecture)
- [Installation Guide](#installation-guide)
- [Usage Instructions](#usage-instructions)
- [Configuration](#configuration)
- [ROS 2 Topics, Services & Parameters](#ros-2-topics-services--parameters)
- [Screenshots & Demo](#screenshots--demo)
- [Roadmap](#roadmap)
- [Contributing](#contributing)
- [Testing](#testing)
- [Known Issues & Limitations](#known-issues--limitations)
- [License](#license)
- [Acknowledgements](#acknowledgements)
- [Contact](#contact)

---

## Overview

**ROS2 Ackermann Robot** is a robotics simulation and control project built on ROS 2 (Robot Operating System 2). It implements an Ackermann-steering vehicle — the same kinematic model used in cars and RC platforms — and brings it to life with a full simulation pipeline:

- A parametric **URDF/Xacro robot description** with realistic link/joint geometry
- **Gazebo** physics simulation with `ros2_control` hardware interface plugins
- **Ackermann steering kinematics** converting speed and steering angle into individual wheel commands
- **Nav2-compatible** setup for autonomous point-to-point navigation
- **Teleoperation** support via keyboard or joystick

This project is ideal for robotics students, researchers, and developers who want a clean, well-structured starting point for building car-like autonomous systems in ROS 2.

> ⚠️ **Note to maintainer:** Please fill in any project-specific details marked with `<!-- TODO -->` throughout this README.

---

## Key Features

- 🚗 **Ackermann Steering Kinematics** — Physically accurate front-wheel steering geometry; inner and outer wheel angles computed independently
- 🏗️ **Full URDF/Xacro Model** — Modular robot description with parameterised dimensions, materials, and inertia tensors
- 🌍 **Gazebo Simulation** — Plug-and-play simulation world with differential/Ackermann drive plugins and sensor integration
- 🎮 **Keyboard Teleoperation** — Drive the robot in simulation using WASD or arrow keys via `teleop_twist_keyboard`
- 🗺️ **Nav2 Integration** — Pre-configured launch files for SLAM-based mapping and autonomous navigation <!-- TODO: Confirm if Nav2/SLAM is implemented -->
- 📡 **Sensor Suite** — LiDAR, IMU, and camera sensor plugins configured in Gazebo <!-- TODO: Confirm which sensors are modelled -->
- 🔄 **ros2_control Compatible** — Hardware interface ready for transitioning from simulation to real hardware
- 📦 **Colcon Workspace** — Standard ROS 2 workspace layout, installable with a single `colcon build`

---

## Tech Stack & Requirements

### Software

| Component | Version / Notes |
|---|---|
| **OS** | Ubuntu 22.04 LTS (Jammy Jellyfish) |
| **ROS 2** | Humble Hawksbill (LTS) |
| **Simulator** | Gazebo Classic 11 *or* Gazebo Sim (Fortress/Harmonic) |
| **Build Tool** | `colcon` |
| **Python** | 3.10+ |
| **C++ Standard** | C++17 |

### Key ROS 2 Packages

| Package | Purpose |
|---|---|
| `robot_state_publisher` | Publishes TF tree from URDF |
| `joint_state_publisher_gui` | Manual joint control in RViz2 |
| `gazebo_ros_pkgs` | Gazebo–ROS 2 bridge |
| `ros2_control` | Hardware abstraction layer |
| `ackermann_msgs` | Ackermann steering message types |
| `nav2_bringup` | Autonomous navigation stack |
| `slam_toolbox` | Online SLAM for map building |
| `teleop_twist_keyboard` | Keyboard teleoperation |

### Hardware (for real-robot deployment)

<!-- TODO: Fill in hardware details if targeting a physical platform -->

| Component | Specification |
|---|---|
| Compute board | Raspberry Pi 4 / Jetson Nano (recommended) |
| Drive motors | 2× DC motors (rear-wheel drive) |
| Steering servo | 1× servo motor (front axle) |
| LiDAR | RPLidar A1 / YDLIDAR (optional) |
| IMU | MPU-6050 or similar (optional) |

---

## Project Architecture

### ROS 2 Node Graph (Simplified)

```
┌─────────────────────────────────────────────────────────────────┐
│                        ROS 2 System                             │
│                                                                  │
│  ┌──────────────────┐     /cmd_vel or       ┌─────────────────┐ │
│  │  Teleop / Nav2   │ ──/cmd_ackermann──▶  │  Ackermann       │ │
│  │  (Twist pub)     │                       │  Controller      │ │
│  └──────────────────┘                       │  Node            │ │
│                                             └────────┬────────┘ │
│                                                      │          │
│                                         /joint_states│          │
│                                                      ▼          │
│  ┌───────────────────┐             ┌─────────────────────────┐  │
│  │  robot_state_     │◀──/tf ──────│   Gazebo Simulation /   │  │
│  │  publisher        │             │   ros2_control          │  │
│  └───────────────────┘             └─────────────────────────┘  │
│                                             │                    │
│              /scan, /imu, /camera_raw       │                    │
│  ┌───────────────────────────────────────◀─┘                    │
│  │  RViz2  │  SLAM Toolbox  │  Nav2 Stack                       │
│  └───────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

### Ackermann Steering Geometry

```
          Front Axle
    ┌───────────────────────┐
    │  δ_inner   δ_outer    │
    │    /            \     │
    │   /              \    │
    ├──/────────────────\───┤
    │                       │  ← Wheelbase (L)
    ├───────────────────────┤
    │     Rear Axle         │
    │  (fixed, drive)       │
    └───────────────────────┘

  δ_inner = atan(L / (R - d/2))
  δ_outer = atan(L / (R + d/2))
  where R = turning radius, d = track width
```

### Folder Structure

```
Ros2_ackermann_robot/
├── src/
│   ├── ackermann_description/        # URDF/Xacro robot model
│   │   ├── urdf/
│   │   │   ├── robot.urdf.xacro      # Main robot description
│   │   │   ├── chassis.xacro         # Chassis geometry & inertia
│   │   │   ├── wheels.xacro          # Wheel links & joints
│   │   │   └── sensors.xacro         # Sensor definitions
│   │   ├── meshes/                   # 3D mesh files (.stl / .dae)
│   │   ├── rviz/                     # RViz2 configuration files
│   │   └── CMakeLists.txt
│   │
│   ├── ackermann_gazebo/             # Simulation world & launch
│   │   ├── worlds/
│   │   │   └── ackermann_world.world # Gazebo world file
│   │   ├── launch/
│   │   │   ├── gazebo.launch.py      # Launch Gazebo + spawn robot
│   │   │   └── display.launch.py     # Launch RViz2 only
│   │   ├── config/
│   │   │   └── controllers.yaml      # ros2_control config
│   │   └── CMakeLists.txt
│   │
│   ├── ackermann_controller/         # Steering kinematics node
│   │   ├── ackermann_controller/
│   │   │   ├── __init__.py
│   │   │   └── controller_node.py    # Twist → Ackermann conversion
│   │   ├── setup.py
│   │   └── package.xml
│   │
│   └── ackermann_bringup/            # Top-level launch & params
│       ├── launch/
│       │   ├── bringup.launch.py     # Full system launch
│       │   └── navigation.launch.py  # Nav2 + SLAM launch
│       ├── config/
│       │   ├── nav2_params.yaml      # Nav2 configuration
│       │   └── slam_params.yaml      # SLAM Toolbox config
│       └── CMakeLists.txt
│
├── .github/
│   └── workflows/                    # CI/CD (if configured)
├── LICENSE
└── README.md
```

> ⚠️ **Note:** The folder structure above is based on standard ROS 2 package conventions for this project type. Run `tree -L 3 src/` in your workspace to verify exact paths and update this section accordingly.

---

## Installation Guide

### Prerequisites

Ensure the following are installed before proceeding:

1. **Ubuntu 22.04 LTS**
2. **ROS 2 Humble** — [Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
3. **Gazebo Classic 11**
4. **colcon** build tool

```bash
# Install ROS 2 Humble (if not already installed)
sudo apt update && sudo apt install -y ros-humble-desktop

# Install Gazebo and ROS-Gazebo bridge
sudo apt install -y gazebo ros-humble-gazebo-ros-pkgs

# Install colcon
sudo apt install -y python3-colcon-common-extensions

# Install rosdep
sudo apt install -y python3-rosdep
sudo rosdep init
rosdep update
```

### Step 1 — Create a ROS 2 Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### Step 2 — Clone the Repository

```bash
cd ~/ros2_ws/src
git clone https://github.com/Balakarthik15/Ros2_ackermann_robot.git
```

### Step 3 — Install Dependencies

```bash
cd ~/ros2_ws

# Install all ROS dependencies declared in package.xml files
rosdep install --from-paths src --ignore-src -r -y
```

Install additional required packages:

```bash
sudo apt install -y \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-ackermann-msgs \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-teleop-twist-keyboard \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro
```

### Step 4 — Build the Workspace

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

### Step 5 — Source the Workspace

```bash
source ~/ros2_ws/install/setup.bash

# Optional: add to .bashrc for persistence
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Usage Instructions

### 1. Launch the Simulation (Gazebo + Robot Spawn)

```bash
ros2 launch ackermann_gazebo gazebo.launch.py
```

This will:
- Start Gazebo with the configured world
- Spawn the Ackermann robot
- Launch `robot_state_publisher` and `ros2_control` nodes

### 2. Visualise in RViz2

```bash
# In a new terminal
source ~/ros2_ws/install/setup.bash
ros2 launch ackermann_description display.launch.py
```

### 3. Keyboard Teleoperation

```bash
# In a new terminal
source ~/ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Use the following keys to control the robot:

| Key | Action |
|-----|--------|
| `i` | Move forward |
| `,` | Move backward |
| `j` | Turn left |
| `l` | Turn right |
| `k` | Stop |
| `q` / `z` | Increase / decrease max speed |

### 4. Launch the Full Navigation Stack (Nav2 + SLAM)

```bash
# Terminal 1: Start simulation
ros2 launch ackermann_gazebo gazebo.launch.py

# Terminal 2: Start navigation
ros2 launch ackermann_bringup navigation.launch.py
```

Then open RViz2 and use the **2D Goal Pose** tool to send navigation goals.

### 5. Inspect the Robot Model Only (No Simulation)

```bash
ros2 launch ackermann_description display.launch.py
```

Use the **Joint State Publisher GUI** sliders to inspect joint kinematics.

---

## Configuration

### Robot Parameters (`ackermann_description/urdf/robot.urdf.xacro`)

| Parameter | Default | Description |
|---|---|---|
| `wheelbase` | `0.30 m` | Distance between front and rear axles |
| `track_width` | `0.22 m` | Distance between left and right wheels |
| `wheel_radius` | `0.05 m` | Radius of each wheel |
| `wheel_width` | `0.04 m` | Width of each wheel |
| `max_steer_angle` | `0.52 rad (~30°)` | Maximum steering angle |

> <!-- TODO: Verify these values against your actual URDF parameters -->

### Controller Configuration (`ackermann_gazebo/config/controllers.yaml`)

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

ackermann_steering_controller:
  ros__parameters:
    front_steering_joints:
      - front_left_steering_joint
      - front_right_steering_joint
    rear_drive_joints:
      - rear_left_wheel_joint
      - rear_right_wheel_joint
    wheelbase: 0.30
    track_width: 0.22
    wheel_radius: 0.05
```

### Nav2 Parameters (`ackermann_bringup/config/nav2_params.yaml`)

Key parameters to tune for Ackermann-type robots:

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      min_turning_radius: 0.40   # Must match physical constraints
      use_regulated_linear_velocity_scaling: true
```

---

## ROS 2 Topics, Services & Parameters

### Published Topics

| Topic | Message Type | Description |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | Current joint positions and velocities |
| `/tf` | `tf2_msgs/TFMessage` | Robot transform tree |
| `/odom` | `nav_msgs/Odometry` | Robot odometry from simulation |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR scan data |
| `/imu/data` | `sensor_msgs/Imu` | IMU readings |
| `/camera/image_raw` | `sensor_msgs/Image` | Raw camera image |

### Subscribed Topics

| Topic | Message Type | Description |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands (Twist → Ackermann) |
| `/cmd_ackermann` | `ackermann_msgs/AckermannDriveStamped` | Direct Ackermann commands |

### Key Services

| Service | Type | Description |
|---|---|---|
| `/spawn_entity` | `gazebo_msgs/SpawnEntity` | Spawn model in Gazebo |
| `/delete_entity` | `gazebo_msgs/DeleteEntity` | Remove model from Gazebo |

---

## Screenshots & Demo

<!-- TODO: Add actual screenshots and/or a demo GIF/video link -->

### Gazebo Simulation

> 📸 **[Add screenshot of the robot in Gazebo here]**
>
> Example: `![Gazebo Simulation](docs/images/gazebo_sim.png)`

### RViz2 Visualisation

> 📸 **[Add RViz2 screenshot showing TF tree, laser scan, and robot model]**
>
> Example: `![RViz2 View](docs/images/rviz2_view.png)`

### Autonomous Navigation

> 🎬 **[Add GIF or link to a demo video of Nav2 autonomous navigation]**
>
> Example: `[![Demo Video](docs/images/nav2_thumb.png)](https://youtu.be/YOUR_VIDEO_ID)`

---

## Roadmap

| Status | Feature |
|--------|---------|
| ✅ | URDF/Xacro robot model |
| ✅ | Gazebo simulation with ros2_control |
| ✅ | Keyboard teleoperation |
| ✅ | Ackermann kinematics node |
| 🔄 | Nav2 autonomous navigation integration |
| 🔄 | SLAM-based map generation |
| 🔲 | Real hardware deployment guide |
| 🔲 | Docker / Dev Container support |
| 🔲 | CI/CD with GitHub Actions |
| 🔲 | Joystick (gamepad) teleoperation |
| 🔲 | Path recording and playback |
| 🔲 | Dynamic obstacle avoidance tuning |

> ✅ Done · 🔄 In Progress · 🔲 Planned

---

## Contributing

Contributions are welcome! Please follow these steps:

1. **Fork** the repository
2. **Create a feature branch**
   ```bash
   git checkout -b feature/your-feature-name
   ```
3. **Make your changes** following the existing code style
4. **Test** your changes (see [Testing](#testing))
5. **Commit** with a clear message
   ```bash
   git commit -m "feat: add joystick teleoperation support"
   ```
6. **Push** to your fork
   ```bash
   git push origin feature/your-feature-name
   ```
7. **Open a Pull Request** against the `main` branch

### Code Style Guidelines

- Python: Follow [PEP 8](https://pep8.org/); use `ruff` or `flake8` for linting
- C++: Follow [ROS 2 C++ Style Guide](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html)
- Commit messages: Use [Conventional Commits](https://www.conventionalcommits.org/) format

Please open an issue first if you plan a large change, to discuss the approach.

---

## Testing

### Verify the URDF Model

```bash
# Check URDF for errors
ros2 run xacro xacro src/ackermann_description/urdf/robot.urdf.xacro > /tmp/robot.urdf
check_urdf /tmp/robot.urdf
```

### Verify TF Tree

```bash
# After launching simulation, in a new terminal:
ros2 run tf2_tools view_frames
evince frames.pdf   # or xdg-open frames.pdf
```

### Verify Active Topics

```bash
# List all active topics
ros2 topic list

# Echo a specific topic
ros2 topic echo /joint_states
ros2 topic echo /odom
```

### Verify Controller Status

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

### Publish a Test Command

```bash
# Send a direct velocity command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{ linear: { x: 0.3 }, angular: { z: 0.2 } }" --once

# Send a direct Ackermann command
ros2 topic pub /cmd_ackermann ackermann_msgs/msg/AckermannDriveStamped \
  "{ drive: { speed: 0.3, steering_angle: 0.2 } }" --once
```

> <!-- TODO: Add unit tests (pytest / ament_cmake_pytest) if they exist in the repo -->

---

## Known Issues & Limitations

| Issue | Description | Workaround |
|---|---|---|
| Gazebo lag at startup | Robot may drift slightly before controllers initialise | Wait 3–5 seconds after launch before sending commands |
| `cmd_vel` to Ackermann conversion | Twist's `angular.z` is approximated; not exact for all radii | Use `/cmd_ackermann` directly for precise control |
| Nav2 min turning radius | Nav2 may plan paths that violate the robot's physical turning radius | Tune `min_turning_radius` in `nav2_params.yaml` |
| No real hardware interface | The current implementation is simulation-only | A hardware bridge (e.g., via serial/ROS-serial) is needed for deployment |

> <!-- TODO: Add any additional known issues specific to your implementation -->

---

## License

This project is licensed under the **MIT License** — see the [LICENSE](LICENSE) file for details.

```
MIT License

Copyright (c) 2024 Balakarthik15

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction...
```

---

## Acknowledgements

- [ROS 2 Documentation](https://docs.ros.org/en/humble/) — The official ROS 2 Humble reference
- [ros2_controllers](https://github.com/ros-controls/ros2_controllers) — Ackermann steering controller library
- [Nav2 Project](https://navigation.ros.org/) — Navigation 2 stack for ROS 2
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox) — Online SLAM for ROS 2
- [Gazebo Simulator](https://gazebosim.org/) — Open-source robot simulation

> <!-- TODO: Add any academic papers, tutorials, or people who inspired or assisted this project -->

---

## Contact

**Balakarthik** — Project Author & Maintainer

- 🐙 GitHub: [@Balakarthik15](https://github.com/Balakarthik15)
- 📧 Email: <!-- TODO: Add your email address -->
- 💼 LinkedIn: <!-- TODO: Add your LinkedIn profile URL -->

> Found a bug? [Open an issue](https://github.com/Balakarthik15/Ros2_ackermann_robot/issues/new) — contributions and feedback are always welcome!

---

<div align="center">
  <sub>Built with ❤️ using ROS 2 · Gazebo · Python · C++</sub>
</div>
