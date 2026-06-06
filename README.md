# 🤖 Transcend — ROS2 Autonomous Routine-Enforcing Rover

> **⚠️ Active Development — Phase 4 of 8 complete.**
> This repository is a living project. New phases are pushed as they're completed.
> See [`Docs/PHASES.md`](Docs/PHASES.md) for the full roadmap.

A 4WD autonomous rover built to **help people who want to help themselves** — enforcing daily routines, detecting doom-scrolling, and physically interrupting unproductive behaviour by driving up to the user and giving a real-world cue.

Built on **ROS2 Jazzy**, **Raspberry Pi 5**, **ESP32**, and **YDLidar X2** — everything from the hardware chassis to the ROS2 control stack was designed and built from scratch.

---

## 📹 Demo Video

[![Demo Video](https://img.shields.io/badge/YouTube-Demo%20Video-red?logo=youtube)](YOUR_YOUTUBE_LINK_HERE)

---

## 📸 Project Photos

> Add your rover photos to the `Images/` folder.

| Rover Build | RViz2 Simulation |
|:-----------:|:----------------:|
| ![Rover](Images/rover_build.jpg) | ![RViz](Images/rviz_simulation.png) |

| Gazebo Simulation | Hardware Wiring |
|:-----------------:|:---------------:|
| ![Gazebo](Images/gazebo_sim.png) | ![Wiring](Images/hardware_wiring.jpg) |

---

## 💡 What is Transcend?

Most productivity tools are passive — they send you a notification you dismiss in 2 seconds. Transcend is **physical**. When you're doom-scrolling past your scheduled work time, it drives to you. You can't swipe it away.

The rover enforces routines by:
- Monitoring your schedule and detecting when you're off-track
- Detecting excessive phone screen time
- **Driving to your location** and providing a physical interruption cue
- Assisting with sleep and wake-up routines
- Doing none of this for people who don't want it — opt-in only

Phase 1–4 of this project establishes the verified motion and navigation foundation. Phase 7 implements the behaviour intelligence layer.

---

## 📊 Project Status

| Phase | Description | Status |
|-------|-------------|--------|
| 1 | Hardware build — 4WD chassis, N20 encoders, ESP32 PID | ✅ Complete |
| 2 | ROS2 workspace + URDF robot description | ✅ Complete |
| 3 | ros2_control hardware interface (RT-safe) | ✅ Complete |
| 4 | Bringup launch + YDLidar + micro-ROS bridge | ✅ Complete |
| 5 | Gazebo Harmonic simulation | 🔄 In Progress |
| 6 | Nav2 navigation + SLAM mapping | 🔲 Planned |
| 7 | Transcend behaviour layer (schedule, doom-scroll, sleep) | 🔲 Planned |
| 8 | Full system integration + mobile app | 🔲 Planned |

---

## ✨ What's Working Right Now

- ✅ Real rover drives in **98% straight line** under PID wheel speed control
- ✅ `diff_drive_controller` receives `/cmd_vel` → splits to 4 independent wheel targets
- ✅ Live encoder feedback published back to ROS2 via `/wheel_vel_state`
- ✅ Full URDF with real physical dimensions (measured from actual rover)
- ✅ Gazebo differential drive simulation running
- ✅ YDLidar X2 scan publishing on `/scan` topic
- ✅ RViz2 visualising robot model and LiDAR scan in real time
- ✅ Single launch file brings up the entire ROS2 stack on Raspberry Pi

---

## 🛠️ Tech Stack

### Software

| Tool | Role |
|------|------|
| **ROS2 Jazzy** | Robot middleware — nodes, topics, services, actions |
| **ros2_control** | Real-time hardware abstraction layer |
| **diff_drive_controller** | Differential drive kinematics + velocity control |
| **Gazebo Harmonic** | Physics simulation |
| **RViz2** | Sensor and robot model visualisation |
| **micro-ROS** | ROS2 on embedded ESP32 (UDP transport) |
| **SLAM Toolbox** *(planned)* | Simultaneous localisation and mapping |
| **Nav2** *(planned)* | Autonomous navigation stack |

### Languages

| Language | Used For |
|----------|---------|
| **C++** | ros2_control hardware interface plugin |
| **Python** | ROS2 nodes, behaviour layer *(planned)* |
| **XML / xacro** | URDF robot description, launch files |
| **YAML** | Controller config, sensor parameters |

### Hardware

| Component | Details |
|-----------|---------|
| Raspberry Pi 5 | Main compute — runs full ROS2 stack |
| ESP32 | Motor controller — runs PID + micro-ROS UDP bridge |
| N20 Encoder Motors ×4 | Geared DC motors with quadrature encoders |
| TB6612FNG H-Bridge ×2 | Motor driver ICs |
| YDLidar X2 | 360° 2D LiDAR — 8m range, 10Hz |
| Camera Module | Visual input *(Phase 7)* |
| Custom Hardware Mount | Aluminium/acrylic base built from scratch |
| Filter Circuit | RC filter + power management for 5V/3.3V rails |
| Li-Po Battery | Main power supply |

---

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Raspberry Pi 5 (ROS2 Jazzy)              │
│                                                             │
│  /cmd_vel (Twist)                                           │
│      ↓                                                      │
│  diff_drive_controller ──────────── /joint_states           │
│      ↓  (wheel velocity targets)         ↑                  │
│  TranscendHardwareInterface    joint_state_broadcaster      │
│      ↓  /wheel_vel_cmd              ↑  /wheel_vel_state     │
│  micro_ros_agent (UDP:8888)         │                       │
│                                     │                       │
│  YDLidar X2 ──→ /scan               │                       │
│  robot_state_publisher ──→ /tf      │                       │
└──────────────────┬──────────────────┼───────────────────────┘
                   │ UDP              │ UDP
┌──────────────────▼──────────────────┴───────────────────────┐
│                      ESP32 (micro-ROS)                      │
│                                                             │
│  Subscribes: /wheel_vel_cmd  → PID setpoints                │
│  Publishes:  /wheel_vel_state ← encoder feedback            │
│                                                             │
│  PID loop per wheel (20Hz) — N20 encoder motors ×4          │
│  TB6612FNG H-bridge driver ×2                               │
└─────────────────────────────────────────────────────────────┘
```

---

## 📦 ROS2 Packages

### `transcend_description`
URDF/xacro robot model built from real measured dimensions.

| File | Description |
|------|-------------|
| `rover.urdf.xacro` | Top-level robot file — includes all sub-files |
| `mobile_base.xacro` | Full robot geometry — links, joints, inertia, ros2_control block |
| `common_properties.xacro` | Reusable inertia macros (box, cylinder, sphere) |
| `rover_gaz.xacro` | Gazebo Harmonic plugins (diff drive + joint state publisher) |
| `ros2_control_snippet.xacro` | Reference snippet for ros2_control block |

**Key dimensions (measured from real rover):**

| Parameter | Value |
|-----------|-------|
| Base length | 0.25 m |
| Base width | 0.14 m |
| Base height | 0.125 m |
| Wheel radius | 0.022 m |
| Wheel separation | 0.158 m |

---

### `transcend_hardware`
Custom `ros2_control` `SystemInterface` plugin — the bridge between ROS2 and ESP32.

**Key design decisions:**
- `write()` runs on the **RT (real-time) thread** — must never block. It deposits velocity commands into `pending_cmd_` protected by a mutex and sets `new_cmd_pending_` atomic flag.
- A **single background thread** does two things: spins the ROS2 executor to receive `/wheel_vel_state` callbacks, and publishes `/wheel_vel_cmd` when the flag is set. DDS network I/O never touches the RT thread.
- Anti-pattern avoided: never calling `publish()` from `write()` directly (would cause RT jitter).

| File | Description |
|------|-------------|
| `include/transcend_hardware_interface.hpp` | Class declaration — 4-joint SystemInterface |
| `src/transcend_hardware_interface.cpp` | Full implementation |
| `WheelVelocity.msg` | Custom message: `fl fr rl rr` float32 fields |
| `transcend_hardware_plugin.xml` | pluginlib export declaration |

**Topics:**

| Topic | Direction | Type | Description |
|-------|-----------|------|-------------|
| `/wheel_vel_cmd` | Pi → ESP32 | `Float32MultiArray` | Velocity setpoints [FL, FR, RL, RR] rad/s |
| `/wheel_vel_state` | ESP32 → Pi | `Float32MultiArray` | Actual encoder velocities [FL, FR, RL, RR] rad/s |

---

### `transcend_bringup`
Launch files and configuration that bring up the full rover stack.

| File | Description |
|------|-------------|
| `launch/rover_pi.launch.xml` | Main launch — runs everything on Pi |
| `config/transcend_controllers.yaml` | diff_drive_controller tuning + limits |
| `config/ydlidar_x4.yaml` | YDLidar X2 sensor config |
| `config/gazebo_bridge.yaml` | ROS↔Gazebo topic bridge mappings |
| `config/transcend_config.rviz` | RViz2 saved config |

---

## ⚙️ Setup & Installation

### Prerequisites

| Requirement | Version |
|-------------|---------|
| Ubuntu | 24.04 LTS |
| ROS2 | Jazzy Jalisco |
| micro-ROS | Jazzy |
| Gazebo | Harmonic |

### 1. Clone the Repository

```bash
git clone https://github.com/YOUR_USERNAME/Transcend-Rover.git
cd Transcend-Rover
```

### 2. Install ROS2 Dependencies

```bash
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Install Additional Dependencies

```bash
# ros2_control
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers

# diff_drive_controller
sudo apt install ros-jazzy-diff-drive-controller ros-jazzy-joint-state-broadcaster

# micro-ROS agent
sudo apt install ros-jazzy-micro-ros-agent

# YDLidar ROS2 driver
# Clone into src/ — see: https://github.com/YDLIDAR/ydlidar_ros2_driver
cd src && git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git
cd ..

# Gazebo Harmonic + bridge
sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge
```

### 4. Build the Workspace

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 5. Launch on Real Rover (Raspberry Pi)

Connect the YDLidar to `/dev/ttyUSB0` and power on the ESP32 with micro-ROS firmware.

```bash
source install/setup.bash
ros2 launch transcend_bringup rover_pi.launch.xml
```

This single command starts:
- `robot_state_publisher`
- `ros2_control` controller manager
- `joint_state_broadcaster` + `diff_drive_controller`
- YDLidar X2 scan driver
- `micro_ros_agent` (UDP port 8888)

### 6. Teleoperate the Rover

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Or publish directly:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}"
```

### 7. Visualise in RViz2

```bash
rviz2 -d src/transcend_bringup/config/transcend_config.rviz
```

---

## 🔑 Key Topics at Runtime

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `Twist` | Drive commands — input to diff_drive_controller |
| `/wheel_vel_cmd` | `Float32MultiArray` | Per-wheel velocity targets → ESP32 |
| `/wheel_vel_state` | `Float32MultiArray` | Per-wheel actual velocity ← ESP32 |
| `/joint_states` | `JointState` | All 4 wheel positions + velocities |
| `/odom` | `Odometry` | Wheel odometry from diff_drive_controller |
| `/scan` | `LaserScan` | 360° LiDAR scan from YDLidar X2 |
| `/tf` | `TFMessage` | Transform tree: odom → base_footprint → base_link |

---

## 📚 Learning Outcomes (So Far)

- ROS2 node, topic, service, and action architecture
- Writing a custom `ros2_control` `SystemInterface` hardware plugin in C++
- RT-safe threading patterns — separating real-time and DDS layers
- URDF/xacro modelling with calculated inertia tensors
- Differential drive kinematics (`wheel_separation`, `wheel_radius` tuning)
- micro-ROS UDP transport — ESP32 as a first-class ROS2 node
- Gazebo Harmonic simulation with ROS2 bridge
- RViz2 configuration for robot visualisation
- Ubuntu system administration and ROS2 workspace management
- colcon build system and ament_cmake package structure

---

## 📁 Repository Structure

```
Transcend-Rover/
│
├── README.md
├── .gitignore
│
├── src/
│   ├── transcend_description/          ← Robot URDF model
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   ├── meshes/
│   │   └── urdf/
│   │       ├── rover.urdf.xacro        ← Top-level robot file
│   │       ├── mobile_base.xacro       ← Geometry + joints + inertia
│   │       ├── common_properties.xacro ← Inertia macros + materials
│   │       ├── rover_gaz.xacro         ← Gazebo plugins
│   │       └── ros2_control_snippet.xacro
│   │
│   ├── transcend_hardware/             ← ros2_control hardware plugin (C++)
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   ├── WheelVelocity.msg
│   │   ├── transcend_hardware_plugin.xml
│   │   ├── include/
│   │   │   └── transcend_hardware_interface.hpp
│   │   └── src/
│   │       └── transcend_hardware_interface.cpp
│   │
│   └── transcend_bringup/              ← Launch files + config
│       ├── package.xml
│       ├── CMakeLists.txt
│       ├── launch/
│       │   └── rover_pi.launch.xml     ← Single launch for full stack
│       └── config/
│           ├── transcend_controllers.yaml
│           ├── ydlidar_x4.yaml
│           ├── gazebo_bridge.yaml
│           └── transcend_config.rviz
│
├── Images/                             ← Add rover photos + screenshots here
│
└── Docs/
    └── PHASES.md                       ← Development phase tracker
```

---

## 🚀 Roadmap

See [`Docs/PHASES.md`](Docs/PHASES.md) for the full detailed phase breakdown.

Short version:
- **Next:** Complete Gazebo LiDAR simulation → Nav2 integration → SLAM mapping
- **Goal:** Fully autonomous rover that detects doom-scrolling and enforces your schedule

---

## 👤 Author

**Mayank Jain**
IoT, Robotics and Automation Enthusiast — B.Tech Robotics & Automation, Bharati Vidyapeeth, Pune

> *ROS2 architecture, hardware interface plugin, URDF modelling, hardware build — all designed and implemented from scratch.*

[![GitHub](https://img.shields.io/badge/GitHub-Profile-black?logo=github)](https://github.com/YOUR_USERNAME)

---

## 📄 License

Apache 2.0 — see [LICENSE](LICENSE)
