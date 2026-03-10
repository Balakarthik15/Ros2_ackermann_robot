# 🤖 ROS 2 Ackermann Robot

> A complete ROS 2 simulation and autonomous navigation stack for a car-like Ackermann-steering robot — featuring a parametric URDF model, Gazebo Harmonic simulation, `gz_ros2_control` hardware interface, GPU LiDAR sensing, SLAM-based mapping, and Nav2 autonomous navigation.

---

[![ROS2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-blue?logo=ros)](https://docs.ros.org/en/jazzy/)
[![Gazebo Harmonic](https://img.shields.io/badge/Gazebo-Harmonic-orange)](https://gazebosim.org/docs/harmonic/)
[![Ubuntu 24.04](https://img.shields.io/badge/Ubuntu-24.04%20LTS-E95420?logo=ubuntu)](https://releases.ubuntu.com/24.04/)
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
- [ROS 2 Topics, Services & TF Tree](#ros-2-topics-services--tf-tree)
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

**ROS 2 Ackermann Robot** is a robotics simulation project built on **ROS 2 Jazzy** and **Gazebo Harmonic**. It models the *Maverick Quantum XT* — a compact, car-like robot using the Ackermann steering geometry found in real vehicles — and brings it to life with a complete simulation and autonomy stack:

- A fully parametric **URDF/Xacro robot model** with physically accurate chassis, wheels, steering links, and an onboard GPU LiDAR
- **Gazebo Harmonic** physics simulation using the `gz_ros2_control` hardware interface plugin
- **Ackermann Steering Controller** (`ros2_controllers`) for independent front-wheel steering and rear-wheel drive
- A custom **keyboard teleoperation** node (`ackermann_teleop`) for real-time manual control
- **SLAM Toolbox** online async mapping to build maps of the simulation environment
- **Nav2** autonomous navigation with a pre-built map and AMCL-ready configuration

This project provides a production-quality, developer-friendly starting point for anyone building car-like autonomous systems in ROS 2.

---

## Key Features

-  **Ackermann Steering Kinematics** — Independent position-controlled front steering joints (`fr_left_steer_joint`, `fr_right_steer_joint`) with ±45° range; rear wheels velocity-controlled for drive
-  **Parametric URDF/Xacro Model** — Full robot geometry with accurate inertia tensors, collision meshes, and Gazebo material properties for chassis, 4 wheels, 2 steering links, virtual steering wheel, and LiDAR
-  **Gazebo Harmonic Simulation** — Spawns into a custom `lab.sdf` world using `ros_gz_sim`; event-driven controller startup sequence prevents race conditions at launch
-  **GPU LiDAR Sensor** — 360° horizontal scan, 10 Hz, 0.15–12 m range, 360 samples/revolution with Gaussian noise, published to `/scan`
-  **Custom Keyboard Teleoperation** — Dedicated `ackermann_teleop` Python package with its own `keyboard_teleop.py` node
-  **Pre-Built Map** — Includes a ready-to-use map (`map.pgm` / `map.yaml` / `map.posegraph`) for instant Nav2 localization
-  **Nav2 Autonomous Navigation** — Full `nav2_bringup` integration with a tuned `nav2_params.yaml` for Ackermann constraints
-  **ros2_control + gz_ros2_control** — Hardware abstraction layer enabling clean simulation-to-hardware portability
-  **Two-Package Workspace** — Clean separation between robot simulation (`ackermann_gazebo`) and teleoperation (`ackermann_teleop`)

---

## Tech Stack & Requirements

### Software

| Component | Version |
|---|---|
| **OS** | Ubuntu 24.04 LTS (Noble Numbat) |
| **ROS 2** | Jazzy Jalisco (LTS) |
| **Simulator** | Gazebo Harmonic |
| **Build Tool** | `colcon` |
| **Python** | 3.12+ |
| **C++ Standard** | C++17 |

### ROS 2 Package Dependencies (from `package.xml`)

| Package | Purpose |
|---|---|
| `robot_state_publisher` | Publishes TF tree from URDF |
| `joint_state_publisher` / `joint_state_publisher_gui` | Joint state visualization |
| `xacro` | URDF macro processing |
| `ros_gz_sim` | Gazebo Harmonic ↔ ROS 2 launch bridge |
| `ros_gz_bridge` | Topic bridging between Gazebo and ROS 2 |
| `ros_gz_image` | Image topic bridge |
| `gz_ros2_control` | Gazebo Harmonic hardware interface for ros2_control |
| `ros2_control` / `controller_manager` | Hardware abstraction and controller lifecycle |
| `ackermann_steering_controller` | Ackermann-specific ros2_controllers plugin |
| `joint_state_broadcaster` | Joint state publisher controller |
| `nav2_bringup` | Full autonomous navigation stack |
| `nav2_amcl` | Adaptive Monte Carlo Localization |
| `nav2_map_server` | Map serving for Nav2 |
| `slam_toolbox` | Online SLAM for map building |
| `tf2`, `tf2_ros`, `tf2_geometry_msgs` | Transform library |
| `rviz2` | 3D visualization |

---

## Project Architecture

### ROS 2 Node & Topic Graph

```
┌──────────────────────────────────────────────────────────────────────┐
│                          ROS 2 System                                │
│                                                                      │
│  ┌─────────────────────┐   /ackermann_steering_controller/reference  │
│  │  ackermann_teleop   │ ─────────────────────────────────────────▶  │
│  │  keyboard_teleop.py │                                             │
│  └─────────────────────┘                                             │
│                                                                      │
│  ┌────────────────────────────────────────────────────────────────┐  │
│  │                  Gazebo Harmonic (lab.sdf)                      │  │
│  │  ┌──────────────────────────────────────────────────────────┐  │  │
│  │  │              bot_ackermann (URDF model)                   │  │  │
│  │  │  chassis_link · fr_left/right_steer_link                  │  │  │
│  │  │  fr/re_left/right_wheel_link                              │  │  │
│  │  │  virtual_steer_link · gpu_lidar link                      │  │  │
│  │  └──────────────────────────────────────────────────────────┘  │  │
│  │     │ /joint_states   │ /scan    │ /odom    │ /tf               │  │
│  └─────┼─────────────────┼──────────┼──────────┼───────────────────┘  │
│        │                 │          │          │                      │
│        ▼                 ▼          ▼          ▼                      │
│  ┌───────────────┐  ┌─────────┐  ┌───────────────────────────────┐   │
│  │ robot_state_  │  │  RViz2  │  │         Nav2 Stack            │   │
│  │ publisher     │  │         │  │  (AMCL + Planner + Controller)│   │
│  └───────────────┘  └─────────┘  └───────────────────────────────┘   │
│        │ /tf                                  │ /cmd_vel              │
│        ▼                                      ▼                      │
│  ┌──────────────┐        ┌────────────────────────────┐              │
│  │ SLAM Toolbox │        │ gz_ros2_control             │              │
│  │  (mapping)   │        │ (hardware bridge)           │              │
│  └──────────────┘        └────────────────────────────┘              │
└──────────────────────────────────────────────────────────────────────┘
```

### Ackermann Steering Geometry

```
               Front Axle   (wheelbase/2 = 0.1425 m ahead of centre)
         ┌──────────────────────────────────────┐
         │  fr_left_steer      fr_right_steer   │
         │      /                      \        │
         │     /    ±45° max steer      \       │
         ├────/────────────────────────── \─────┤
         │                                      │  Wheelbase = 0.285 m
         ├──────────────────────────────────────┤
         │   re_left_wheel       re_right_wheel  │
         │           (rear axle — drive)         │
         └──────────────────────────────────────┘
                     Track Width = 0.280 m
```

### Robot Physical Parameters (from `bot_sample.xacro`)

| Parameter | Value |
|---|---|
| Robot Name | `maverick_quantum_xt` |
| Body Length | 0.475 m |
| Body Width | 0.210 m |
| Body Height | 0.160 m |
| Body Mass | 1.58 kg |
| Wheelbase | 0.285 m |
| Track Width | 0.280 m |
| Wheel Radius | 0.055 m |
| Wheel Thickness | 0.070 m |
| Wheel Mass | 0.12 kg |
| Steering Link Mass | 0.02 kg |
| Max Steering Angle | ±45° (π/4 rad) |
| LiDAR Radius | 0.035 m |
| LiDAR Height | 0.055 m |
| LiDAR Mass | 0.05 kg |

### Folder Structure

```
Ros2_ackermann_robot/
└── src/
    ├── ackermann_gazebo/                    # Main simulation package (ament_cmake)
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── urdf/
    │   │   ├── vehicle.urdf.xacro           # Top-level robot entry point
    │   │   ├── bot.xacro                    # Core robot structure
    │   │   ├── bot_sample.xacro             # Full parametric robot model
    │   │   └── gz_ackermann_core.xacro      # Gazebo plugin definitions
    │   ├── launch/
    │   │   ├── robot.launch.py              # Gazebo simulation launch
    │   │   └── navigation.launch.py         # Nav2 navigation launch
    │   ├── config/
    │   │   ├── gz_ros2_control.yaml         # ros2_control controller config
    │   │   ├── nav2_params.yaml             # Nav2 stack parameters
    │   │   ├── mapper_params_online_async.yaml  # SLAM Toolbox config
    │   │   ├── robot_params.yaml            # Robot-level parameters
    │   │   └── ros_gz_bridge.yaml           # Gazebo ↔ ROS 2 topic bridge
    │   ├── worlds/
    │   │   └── lab.sdf                      # Simulation world (SDF format)
    │   ├── maps/
    │   │   ├── map.pgm                      # Pre-built occupancy grid image
    │   │   ├── map.yaml                     # Map metadata
    │   │   ├── map.data                     # SLAM Toolbox map data
    │   │   └── map.posegraph                # SLAM pose graph
    │   ├── meshes/
    │   │   └── lidar/                       # LiDAR mesh assets
    │   ├── include/
    │   │   └── ackermann_gazebo/            # C++ headers (reserved)
    │   └── src/                             # C++ source (reserved)
    │
    └── ackermann_teleop/                    # Keyboard teleoperation (ament_python)
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        ├── resource/
        │   └── ackermann_teleop
        ├── ackermann_teleop/
        │   ├── __init__.py
        │   └── keyboard_teleop.py           # Keyboard teleoperation node
        └── test/
            ├── test_copyright.py
            ├── test_flake8.py
            ├── test_pep257.py
            └── test_xmllint.py
```

---

## Installation Guide

### Prerequisites

Ensure the following are installed on **Ubuntu 24.04 LTS**.

#### 1. Install ROS 2 Jazzy

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu noble main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-jazzy-desktop
```

#### 2. Install Gazebo Harmonic + ROS Bridge

```bash
sudo apt install -y ros-jazzy-ros-gz
```

#### 3. Install Build Tools

```bash
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool
sudo rosdep init
rosdep update
```

---

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

### Step 3 — Install ROS Dependencies

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

Install additional controller and navigation packages manually:

```bash
sudo apt install -y \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ackermann-steering-controller \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-amcl \
  ros-jazzy-nav2-map-server \
  ros-jazzy-slam-toolbox \
  ros-jazzy-xacro \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-rviz2
```

### Step 4 — Build the Workspace

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

### Step 5 — Source the Workspace

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Persist across terminals
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Usage Instructions

### 1. Launch the Robot Simulation (Gazebo Harmonic)

```bash
ros2 launch ackermann_gazebo robot.launch.py
```

This single launch file performs the full startup sequence:

1. Kills any lingering `gz` processes
2. Launches **Gazebo Harmonic** with `lab.sdf`
3. Processes `vehicle.urdf.xacro` and spawns `bot_ackermann`
4. Starts `robot_state_publisher` and `ros_gz_bridge`
5. Activates `joint_state_broadcaster` (triggered after spawn completes)
6. Activates `ackermann_steering_controller` (triggered after joint broadcaster is active)

Override spawn position with launch arguments:

```bash
ros2 launch ackermann_gazebo robot.launch.py \
  x:=1.0 y:=2.0 z:=0.2 Y:=1.57
```

### 2. Keyboard Teleoperation

In a new terminal:

```bash
ros2 run ackermann_teleop keyboard_teleop
```

The node sends commands directly to the Ackermann steering controller. Key bindings are printed to the terminal on startup.

### 3. Visualise in RViz2

```bash
rviz2
```

Recommended displays to add:

| Display | Topic |
|---|---|
| RobotModel | `/robot_description` |
| TF | *(auto)* |
| LaserScan | `/scan` |
| Odometry | `/odom` |
| Map | `/map` *(Nav2 only)* |

### 4. Build a Map with SLAM Toolbox

```bash
# Terminal 1: Start simulation
ros2 launch ackermann_gazebo robot.launch.py

# Terminal 2: Start SLAM Toolbox
ros2 launch slam_toolbox online_async_launch.py \
  params_file:=src/ackermann_gazebo/config/mapper_params_online_async.yaml \
  use_sim_time:=true

# Terminal 3: Drive around to explore the environment
ros2 run ackermann_teleop keyboard_teleop

# Terminal 4: Save the map when satisfied
ros2 run nav2_map_server map_saver_cli \
  -f src/ackermann_gazebo/maps/map
```

### 5. Autonomous Navigation with Nav2 (Pre-Built Map)

A complete map is already included under `ackermann_gazebo/maps/`.

```bash
# Terminal 1: Start simulation
ros2 launch ackermann_gazebo robot.launch.py

# Terminal 2: Start Nav2
ros2 launch ackermann_gazebo navigation.launch.py
```

Then in **RViz2**:
1. Click **2D Pose Estimate** → click on the map to set the robot's initial position
2. Click **2D Goal Pose** → click anywhere on the map to send a navigation goal

---

## Configuration

### ros2_control Controllers (`config/gz_ros2_control.yaml`)

The `ackermann_steering_controller` manages:
- **Front steering joints** (`fr_left_steer_joint`, `fr_right_steer_joint`) — position command interface
- **Rear drive wheels** (`re_left_wheel_joint`, `re_right_wheel_joint`) — velocity command interface

Key parameters to verify match your URDF values:

```yaml
ackermann_steering_controller:
  ros__parameters:
    wheelbase: 0.285       # metres — must match URDF
    track_width: 0.280     # metres — must match URDF
    wheel_radius: 0.055    # metres — must match URDF
```

### Gazebo ↔ ROS 2 Bridge (`config/ros_gz_bridge.yaml`)

Defines topic bridging between Gazebo Harmonic and ROS 2. Add new sensor or plugin topics here as the robot model grows.

### Nav2 Parameters (`config/nav2_params.yaml`)

Key tuning note for Ackermann robots — the minimum turning radius is a hard physical constraint:

```
min_turning_radius = wheelbase / tan(max_steering_angle)
                   = 0.285 / tan(45°)
                   ≈ 0.285 m
```

Ensure this value is set correctly in the Nav2 controller plugin configuration to prevent the planner from generating infeasible paths.

### SLAM Toolbox (`config/mapper_params_online_async.yaml`)

Configured for online asynchronous mapping. Always ensure:

```yaml
use_sim_time: true   # Required when running with Gazebo
```

### Robot Parameters (`config/robot_params.yaml`)

Stores runtime-accessible robot-level parameters. Update this file when changing physical robot dimensions or sensor configuration.

---

## ROS 2 Topics, Services & TF Tree

### Published Topics

| Topic | Message Type | Source | Description |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | Gazebo JointStatePublisher plugin | All 7 joint positions and velocities at 100 Hz |
| `/tf` / `/tf_static` | `tf2_msgs/TFMessage` | `robot_state_publisher` + controllers | Full transform tree |
| `/odom` | `nav_msgs/Odometry` | `ackermann_steering_controller` | Wheel odometry |
| `/scan` | `sensor_msgs/LaserScan` | GPU LiDAR (Gazebo) | 360° LiDAR at 10 Hz, 0.15–12 m range |
| `/robot_description` | `std_msgs/String` | `robot_state_publisher` | Processed URDF string |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM Toolbox / `map_server` | Occupancy grid map |

### Subscribed Topics

| Topic | Message Type | Subscriber | Description |
|---|---|---|---|
| `/ackermann_steering_controller/reference` | `ackermann_msgs/AckermannDrive` | Ackermann controller | Speed + steering angle commands |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 → controller bridge | Velocity commands from Nav2 planner |

### Active Controllers

```bash
# Verify at runtime after launching simulation:
ros2 control list_controllers
```

| Controller | Type | State |
|---|---|---|
| `joint_state_broadcaster` | `JointStateBroadcaster` | active |
| `ackermann_steering_controller` | `AckermannSteeringController` | active |

### TF Tree (from `view_frames` output)

```
odom  (50.5 Hz)
└── base_footprint
    └── base_link
        └── chassis_link  (static)
            ├── bot_ackermann/base_footprint/gpu_lidar  (static)
            ├── fr_left_steer_link   (20.5 Hz)
            │   └── fr_left_wheel_link
            ├── fr_right_steer_link  (20.5 Hz)
            │   └── fr_right_wheel_link
            ├── re_left_wheel_link   (20.5 Hz)
            ├── re_right_wheel_link  (20.5 Hz)
            └── virtual_steer_link   (20.5 Hz)
```

---


## Roadmap

| Status | Feature |
|--------|---------|
| ✅ | Parametric URDF/Xacro robot model (`maverick_quantum_xt`) |
| ✅ | Gazebo Harmonic simulation with `gz_ros2_control` |
| ✅ | Ackermann Steering Controller (position steering + velocity drive) |
| ✅ | GPU LiDAR sensor (360°, 10 Hz, `/scan`) |
| ✅ | Custom keyboard teleoperation node (`ackermann_teleop`) |
| ✅ | Pre-built SLAM map included (`maps/`) |
| ✅ | Nav2 bringup integration |
| ✅ | SLAM Toolbox online async mapping |
| 🔄 | RViz2 `.rviz` config file for one-click visualization |
| 🔄 | Fix hardcoded paths in `navigation.launch.py` |
| 🔄 | Joystick / gamepad teleoperation |
| 🔲 | Real hardware deployment guide (Raspberry Pi / Jetson) |
| 🔲 | Docker / Dev Container support |
| 🔲 | GitHub Actions CI/CD pipeline |
| 🔲 | Camera sensor plugin and image topic integration |
| 🔲 | Path recording and playback node |

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
4. **Run lint tests** (see [Testing](#testing))
5. **Commit** with a descriptive message
   ```bash
   git commit -m "feat: add joystick teleoperation support"
   ```
6. **Push** to your fork and **open a Pull Request** against `main`

### Code Style Guidelines

- **Python**: Follow [PEP 8](https://pep8.org/) — enforced by `test_flake8.py` and `test_pep257.py`
- **C++**: Follow the [ROS 2 C++ Style Guide](https://docs.ros.org/en/jazzy/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html)
- **Commit messages**: Use [Conventional Commits](https://www.conventionalcommits.org/) (`feat:`, `fix:`, `docs:`, `chore:`)
- **Copyright headers**: Required in all source files — enforced by `test_copyright.py`

Please open an issue before starting large changes to discuss the approach.

---

## Testing

### Lint & Style Tests (`ackermann_teleop`)

The `ackermann_teleop` package includes automated tests covering style and XML validation:

```bash
cd ~/ros2_ws
colcon test --packages-select ackermann_teleop
colcon test-result --verbose
```

This runs:

| Test File | Checks |
|---|---|
| `test_copyright.py` | Copyright headers present in all source files |
| `test_flake8.py` | Python style and error linting (PEP 8) |
| `test_pep257.py` | Python docstring conventions |
| `test_xmllint.py` | XML and launch file validity |

### Validate the URDF

```bash
ros2 run xacro xacro \
  src/ackermann_gazebo/urdf/vehicle.urdf.xacro > /tmp/robot.urdf
check_urdf /tmp/robot.urdf
```

### Verify TF Tree at Runtime

```bash
# After launching simulation in another terminal:
ros2 run tf2_tools view_frames
# Opens frames.pdf — should match the TF tree documented above
```

### Verify Controller Status

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

### Send Test Commands

```bash
# Send a direct Ackermann command
ros2 topic pub /ackermann_steering_controller/reference \
  ackermann_msgs/msg/AckermannDrive \
  "{ speed: 0.3, steering_angle: 0.3 }" --once

# Verify odometry response
ros2 topic echo /odom --once

# Check LiDAR publish rate
ros2 topic hz /scan
```

---

## Known Issues & Limitations

| Issue | Description | Workaround / Fix |
|---|---|---|
| **Hardcoded package path** | `navigation.launch.py` uses `pkg_ackermann_bringup = "/home/karthik/..."` — an absolute path that breaks on other machines | Replace with `get_package_share_directory('ackermann_gazebo')` |
| **Commented-out AMCL / map_server nodes** | `amcl_node` and `map_server_node` in `navigation.launch.py` are commented out | Uncomment and configure when switching from SLAM to AMCL localisation |
| **`pkill -9 gz` on startup** | `robot.launch.py` kills all `gz` processes before launching — may affect unrelated Gazebo instances | Remove `cleanup_gz` action for multi-simulation or production environments |
| **Simulation-only** | The hardware plugin is `gz_ros2_control/GazeboSimSystem` — no real hardware interface exists yet | A serial or CAN hardware bridge would be needed for physical deployment |
| **`virtual_steer_link` not actuated** | `virtual_steer_link` is a visual-only element and is not part of the `ros2_control` interface | By design; represents a display element — can be hidden in RViz2 if needed |
| **No RViz2 config file** | The repository does not include a `.rviz` config file — displays must be set up manually | Planned for a future release |

---

## License

<!-- TODO: Add a LICENSE file to the repository root and update package.xml -->

This project does not yet declare an explicit license in `package.xml`. It is recommended to add an open-source license before public release.

Suggested steps:
1. Choose a license — [MIT](https://opensource.org/licenses/MIT) or [Apache 2.0](https://www.apache.org/licenses/LICENSE-2.0) are common in ROS 2 projects
2. Add a `LICENSE` file to the repository root
3. Update `package.xml`: `<license>MIT</license>`
4. Add copyright headers to all source files (required by `test_copyright.py`)

---

## Acknowledgements

- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/) — Official ROS 2 Jazzy reference
- [Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic/) — Next-generation Gazebo simulator
- [gz_ros2_control](https://github.com/ros-controls/gz_ros2_control) — Gazebo Harmonic hardware interface for ros2_control
- [ros2_controllers — Ackermann Steering Controller](https://control.ros.org/master/doc/ros2_controllers/ackermann_steering_controller/doc/userdoc.html)
- [Nav2 Project](https://navigation.ros.org/) — Navigation 2 stack for ROS 2
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox) — Online SLAM for ROS 2 by Steve Macenski

---

## Contact

**Karthik** — Project Author & Maintainer

- 🐙 GitHub: [@Balakarthik15](https://github.com/Balakarthik15)
- 📧 Email: [msbkarthik1511@gmail.com](mailto:msbkarthik1511@gmail.com)
- 💼 LinkedIn: [linkedin.com/in/balakarthiksenthilvelpalani](https://www.linkedin.com/in/balakarthiksenthilvelpalani/)

> Found a bug or have a feature request? [Open an issue](https://github.com/Balakarthik15/Ros2_ackermann_robot/issues/new) — contributions and feedback are always welcome!

---

<div align="center">
  <sub>Built with ❤️ using ROS 2 Jazzy · Gazebo Harmonic · Python · C++</sub>
</div>
