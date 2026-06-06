# 🤖 Transcend — ROS2 Autonomous Routine-Enforcing Rover

> **⚠️ Active Development — Phase 5 complete, Phase 6 (real hardware integration) in progress.**
> This repository is a living project. Phases are pushed as milestones are completed.
> See [`Docs/PHASES.md`](Docs/PHASES.md) for the full roadmap.

A 4WD autonomous rover built to **help people who want to help themselves** — enforcing daily schedules, detecting doom-scrolling, and physically driving to the user as a real-world interruption cue.

Built on **ROS2 Jazzy**, **Raspberry Pi 4B**, **ESP32**, and **YDLidar X2** — every layer from the physical chassis to the ROS2 control stack was designed and built from scratch.

---

## 📹 Demo Video

[![Demo Video](https://img.shields.io/badge/YouTube-Demo%20Video-red?logo=youtube)](YOUR_YOUTUBE_LINK_HERE)

---

## 📸 Project Photos

> Add your photos to the `Images/` folder and update paths below.

| Rover Build | Hardware Mount |
|:-----------:|:--------------:|
| ![Rover](Images/rover_build.jpeg) | ![Mount](Images/hardware_mount.jpeg) |

| RViz2 Visualisation |
|:-------------------:|
| ![RViz](Images/rviz_simulation.jpeg) |

---

## 💡 What is Transcend?

Most productivity tools are passive — you dismiss the notification in 2 seconds. Transcend is **physical**. When you're doom-scrolling past your scheduled work time, it drives to you. You can't swipe it away.

The rover enforces routines by:
- Monitoring your schedule and detecting when you're off-track
- Detecting excessive phone screen time
- **Driving to your location** and providing a physical interruption cue
- Assisting with sleep and wake-up routines
- Doing none of this for people who don't want it — **opt-in by design**

Phases 1–5 establish the verified motion and simulation foundation. Phase 6 is integrating everything on real hardware. Phase 7 brings autonomous navigation. Phase 8 brings the behaviour intelligence.

---

## 📊 Project Status

| Phase | Description | Status |
|-------|-------------|--------|
| 1 | Hardware build — 4WD chassis, N20 encoders, filter circuit, custom mount | ✅ Complete |
| 2 | ROS2 workspace + URDF robot description | ✅ Complete |
| 3 | ros2_control hardware interface plugin (C++, RT-safe) | ✅ Complete |
| 4 | Bringup launch + YDLidar + micro-ROS bridge | ✅ Complete |
| 5 | Gazebo Harmonic simulation | ✅ Complete |
| 6 | Real hardware integration — Pi 4B + ESP32 + LiDAR running together | 🔄 In Progress |
| 7 | SLAM Toolbox + Nav2 autonomous navigation | 🔲 Planned |
| 8 | Transcend behaviour layer — schedule enforcement, doom-scroll detection | 🔲 Planned |
| 9 | AI/ML integration — person tracking, computer vision | 🔲 Planned |
| 10 | Full system integration + mobile companion app | 🔲 Planned |

---

## ✨ What's Working Right Now

- ✅ Real rover drives in **98% straight line** under PID wheel speed control
- ✅ `diff_drive_controller` receives `/cmd_vel` → splits to 4 independent wheel targets
- ✅ Encoder feedback published back to ROS2 via `/wheel_vel_state`
- ✅ Full URDF with real physical dimensions (measured from actual rover)
- ✅ **Gazebo Harmonic simulation fully working** — differential drive, joint states, LiDAR
- ✅ YDLidar X2 scan publishing on `/scan` topic
- ✅ RViz2 visualising robot model and LiDAR scan in real time
- ✅ Single launch file brings up the entire ROS2 stack on Raspberry Pi
- ✅ ESP32 micro-ROS firmware — **full ROS2 node on microcontroller** with watchdog, deadband fix, and direction-aware encoder ISRs
- 🔄 Full hardware stack integration (Pi 4B ↔ ESP32 micro-ROS bridge) in progress

---

## 🛠️ Tech Stack

### Software

| Tool | Role |
|------|------|
| **ROS2 Jazzy** | Robot middleware — nodes, topics, services, actions |
| **ros2_control** | Real-time hardware abstraction layer |
| **diff_drive_controller** | Differential drive kinematics + velocity control |
| **micro-ROS** | Full ROS2 node running on ESP32 over WiFi UDP |
| **Gazebo Harmonic** | Physics simulation ✅ working |
| **RViz2** | Sensor and robot model visualisation |
| **SLAM Toolbox** | Simultaneous localisation and mapping *(Phase 7)* |
| **Nav2** | Autonomous navigation stack *(Phase 7)* |

### Languages

| Language | Used For |
|----------|---------|
| **C++** | ros2_control hardware interface plugin |
| **Arduino C++** | ESP32 micro-ROS firmware — PID, encoder ISRs, state machine |
| **Python** | ROS2 utility nodes, behaviour layer *(Phase 8)* |
| **XML / xacro** | URDF robot description, launch files |
| **YAML** | Controller config, sensor parameters |

### Hardware

| Component | Details |
|-----------|---------|
| **Raspberry Pi 4B** | Main compute — runs full ROS2 Jazzy + micro-ROS agent |
| **ESP32** | Dedicated motor controller — runs as a full ROS2 node via micro-ROS |
| N20 Encoder Motors ×4 | Geared DC motors with quadrature encoders |
| TB6612FNG H-Bridge ×2 | Motor driver ICs (front pair + rear pair) |
| YDLidar X2 | 360° 2D LiDAR — 8m range, 10Hz |
| Camera Module | Visual input *(Phase 9)* |
| Custom Hardware Mount | Built from scratch — camera, LiDAR, and Pi all housed |
| Filter Circuit | RC filter + power management for 5V/3.3V rails |
| Li-Po Battery | Main power supply |

---

## 🏗️ System Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                   Raspberry Pi 4B (ROS2 Jazzy)                   │
│                                                                  │
│  /cmd_vel (Twist)                                                │
│       ↓                                                          │
│  diff_drive_controller ───────────────  /joint_states            │
│       ↓  (4× wheel velocity rad/s)            ↑                  │
│  TranscendHardwareInterface       joint_state_broadcaster        │
│       ↓  /wheel_vel_cmd [FL,FR,RL,RR]     ↑  /wheel_vel_state    │
│  micro_ros_agent (UDP port 8888)         │                       │
│                                          │                       │
│  YDLidar X2 driver ──→ /scan             │                       │
│  robot_state_publisher ──→ /tf           │                       │
└──────────────────┬───────────────────────┼───────────────────────┘
                   │ WiFi UDP              │ WiFi UDP
┌──────────────────▼───────────────────────┴──────────────────────┐
│                  ESP32 (micro-ROS node: transcend_esp32)        │
│                                                                 │
│  Subscriber: /wheel_vel_cmd  → float32[FL, FR, RL, RR] rad/s    │
│  Publisher:  /wheel_vel_state ← float32[FL, FR, RL, RR] rad/s   │
│                                                                 │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │  PID loop — 50 Hz (20ms interval)                       │    │
│  │  • Direction-aware quadrature encoder ISRs              │    │
│  │  • PWM deadband (30/255) to overcome static friction    │    │
│  │  • Direction-change integrator reset                    │    │
│  │  • 2s watchdog — stops motors if ROS2 comms lost        │    │
│  └─────────────────────────────────────────────────────────┘    │
│                                                                 │
│  Agent state machine: WAITING_WIFI → WAITING_AGENT → CONNECTED  │
└─────────────────────────────────────────────────────────────────┘
```

---

## 📦 ROS2 Packages

### `transcend_description`
URDF/xacro robot model built from real measured rover dimensions.

| File | Description |
|------|-------------|
| `rover.urdf.xacro` | Top-level robot file |
| `mobile_base.xacro` | Full geometry — links, joints, inertia, ros2_control block |
| `common_properties.xacro` | Reusable inertia macros (box, cylinder, sphere) |
| `rover_gaz.xacro` | Gazebo Harmonic plugins (diff drive + joint state publisher) |
| `ros2_control_snippet.xacro` | ros2_control hardware block |

**Real measured dimensions:**

| Parameter | Value |
|-----------|-------|
| Base length | 0.25 m |
| Base width | 0.14 m |
| Base height | 0.125 m |
| Wheel radius | 0.022 m |
| Wheel separation | 0.158 m |

---

### `transcend_hardware`
Custom `ros2_control` `SystemInterface` plugin — the Pi-side bridge between ROS2 controllers and the ESP32.

**Key design decisions:**
- `write()` runs on the **RT thread** — it only deposits velocity commands into `pending_cmd_` behind a mutex and sets an atomic flag. No DDS calls, no blocking.
- A **single background thread** handles both DDS subscriber callbacks (`/wheel_vel_state`) and publisher calls (`/wheel_vel_cmd`). Network I/O never touches the RT thread.
- Avoids the common anti-pattern of calling `publish()` directly from `write()`, which causes RT jitter.

| File | Description |
|------|-------------|
| `include/transcend_hardware_interface.hpp` | Class declaration — 4-joint SystemInterface |
| `src/transcend_hardware_interface.cpp` | Full RT-safe implementation |
| `WheelVelocity.msg` | Custom message — fl fr rl rr float32 fields |
| `transcend_hardware_plugin.xml` | pluginlib export |

---

### `transcend_bringup`
Single launch file that starts the entire rover stack.

| File | Description |
|------|-------------|
| `launch/rover_pi.launch.xml` | Starts RSP, controller manager, LiDAR driver, micro-ROS agent |
| `config/transcend_controllers.yaml` | diff_drive_controller gains + kinematic limits |
| `config/ydlidar_x4.yaml` | YDLidar X2 sensor parameters |
| `config/gazebo_bridge.yaml` | ROS ↔ Gazebo topic bridge |
| `config/transcend_config.rviz` | Saved RViz2 configuration |

---

### `ESP32_Firmware` — micro-ROS Motor Controller
The ESP32 runs as a **full ROS2 node** using micro-ROS over WiFi UDP. It is not a simple serial bridge — it subscribes and publishes ROS2 topics directly.

**What it does:**

| Feature | Detail |
|---------|--------|
| ROS2 node name | `transcend_esp32` |
| Subscriber | `/wheel_vel_cmd` — `Float32MultiArray` [FL, FR, RL, RR] in rad/s |
| Publisher | `/wheel_vel_state` — `Float32MultiArray` [FL, FR, RL, RR] in rad/s |
| PID rate | 50 Hz (20ms interval) |
| State machine | `WAITING_WIFI → WAITING_AGENT → AGENT_CONNECTED` |
| Watchdog | 2000ms — stops motors if `/wheel_vel_cmd` stops arriving |
| PWM deadband | 30/255 minimum — ensures motors overcome static friction |
| Encoder ISRs | Direction-aware quadrature decoding (signed tick counting) |
| Direction reset | PID integrator resets on direction reversal to prevent windup |
| Agent ping | Lightweight (100ms, 1 attempt) — non-blocking connection check every 5s |

**Physical constants used in firmware:**

| Constant | Value | Description |
|----------|-------|-------------|
| `WHEEL_RADIUS_M` | 0.022 m | Matches URDF |
| `WHEEL_SEPARATION_M` | 0.158 m | Matches URDF |
| `TICKS_PER_REV` | 1395 | Measured from N20 encoder |
| `MAX_SPEED_TICKS` | 2865 | Calibrated ticks/sec at full PWM |
| `TICKS_PER_RAD` | ≈ 222 | Derived from TICKS_PER_REV / 2π |

**Bug fixes documented in firmware (v — FIXED):**
1. Debug `printf` moved inside `AGENT_CONNECTED` case — was floating between case blocks causing undefined behaviour in C++
2. Executor spin window increased 1ms → 15ms for reliable WiFi UDP packet pickup
3. Watchdog bumped to 2000ms — headroom for reconnects (commands arrive every 20ms at 50Hz)
4. Agent ping reduced to (100ms, 1 attempt) — 10× less blocking than before
5. PWM deadband added (30/255) — at low speeds initial PWM was ~13 (5%), below motor stall threshold
6. Encoder ISRs now do signed counting (direction-aware) instead of increment-only

> ⚠️ **Before uploading the firmware:** fill in `YOUR_WIFI_SSID`, `YOUR_WIFI_PASSWORD`, and `YOUR_AGENT_IP` (your Raspberry Pi's local IP) at the top of the `.ino` file.
> Run `hostname -I` on the Pi to find its IP address.

---

## ⚙️ Setup & Installation

### Prerequisites

| Requirement | Version |
|-------------|---------|
| Ubuntu | 24.04 LTS |
| ROS2 | Jazzy Jalisco |
| Gazebo | Harmonic |
| ESP32 Arduino Core | 3.x |
| micro-ROS Arduino library | Jazzy branch |

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

### 3. Install Additional Packages

```bash
# ros2_control stack
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers

# diff_drive_controller + joint_state_broadcaster
sudo apt install ros-jazzy-diff-drive-controller ros-jazzy-joint-state-broadcaster

# micro-ROS agent (runs on Pi, bridges ESP32 ↔ ROS2)
sudo apt install ros-jazzy-micro-ros-agent

# Gazebo Harmonic + ROS bridge
sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge

# YDLidar ROS2 driver
cd src && git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git && cd ..
```

### 4. Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 5. Launch on Real Rover (Raspberry Pi 4B)

```bash
source install/setup.bash
ros2 launch transcend_bringup rover_pi.launch.xml
```

This single command starts robot_state_publisher, controller manager, joint_state_broadcaster, diff_drive_controller, YDLidar X2 driver, and micro_ros_agent.

### 6. Teleoperate

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 7. Visualise in RViz2

```bash
rviz2 -d src/transcend_bringup/config/transcend_config.rviz
```

### 8. ESP32 Firmware Setup

1. Install the **micro-ROS Arduino library** (Jazzy branch) from: https://github.com/micro-ROS/micro_ros_arduino
2. Open `ESP32_Firmware/transcend_esp32_microros/transcend_esp32_microros.ino` in Arduino IDE
3. Fill in your credentials at the top of the file:
   ```cpp
   const char* WIFI_SSID     = "YOUR_WIFI_SSID";
   const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
   const char* AGENT_IP      = "YOUR_PI_IP";   // run: hostname -I on the Pi
   const int   AGENT_PORT    = 8888;
   ```
4. Select board: **ESP32 Dev Module** and your COM port
5. Upload via USB
6. Open Serial Monitor at **115200 baud** — you should see `[micro_ROS] Agent found!` once the Pi's micro_ros_agent is running

---

## 🔑 Key Topics at Runtime

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `Twist` | Drive commands into diff_drive_controller |
| `/wheel_vel_cmd` | `Float32MultiArray` | Per-wheel velocity targets → ESP32 (rad/s) |
| `/wheel_vel_state` | `Float32MultiArray` | Per-wheel actual velocity ← ESP32 (rad/s) |
| `/joint_states` | `JointState` | All 4 wheel positions + velocities |
| `/odom` | `Odometry` | Wheel odometry from diff_drive_controller |
| `/scan` | `LaserScan` | 360° LiDAR scan from YDLidar X2 |
| `/tf` | `TFMessage` | odom → base_footprint → base_link transform tree |

---

## 📁 Repository Structure

```
Transcend-Rover/
│
├── README.md
├── LICENSE                               ← Apache 2.0
├── .gitignore                            ← excludes build/, install/, log/
├── CONTRIBUTING.md
├── .github/ISSUE_TEMPLATE/
│
├── src/
│   ├── transcend_description/            ← URDF/xacro robot model
│   │   ├── urdf/
│   │   │   ├── rover.urdf.xacro
│   │   │   ├── mobile_base.xacro
│   │   │   ├── common_properties.xacro
│   │   │   ├── rover_gaz.xacro
│   │   │   └── ros2_control_snippet.xacro
│   │   └── meshes/
│   │
│   ├── transcend_hardware/               ← C++ ros2_control plugin (RT-safe)
│   │   ├── include/transcend_hardware_interface.hpp
│   │   ├── src/transcend_hardware_interface.cpp
│   │   └── WheelVelocity.msg
│   │
│   └── transcend_bringup/               ← Launch files + all config
│       ├── launch/rover_pi.launch.xml
│       └── config/
│           ├── transcend_controllers.yaml
│           ├── ydlidar_x4.yaml
│           ├── gazebo_bridge.yaml
│           └── transcend_config.rviz
│
├── ESP32_Firmware/
│   └── transcend_esp32_microros/
│       └── transcend_esp32_microros.ino  ← micro-ROS firmware (full ROS2 node)
│
├── Images/
│   ├── hardware_mount.jpeg
│   ├── rover_build.jpeg
│   └── rviz_simulation.jpeg       
│
└── Docs/
    └── PHASES.md                        ← 10-phase development tracker
```

---

## 📚 Learning Outcomes (So Far)

- ROS2 node, topic, service, and action architecture
- Writing a custom `ros2_control` `SystemInterface` plugin in C++
- RT-safe threading — separating real-time and DDS I/O layers
- URDF/xacro modelling with calculated inertia tensors
- Differential drive kinematics and odometry
- micro-ROS — running a full ROS2 node on an ESP32 microcontroller over WiFi UDP
- State machine design for robust embedded agent connection handling
- PID tuning with anti-windup, deadband compensation, and direction-change reset
- Direction-aware quadrature encoder decoding via ISR
- Gazebo Harmonic simulation with ROS2 bridge
- RViz2 visualisation and configuration
- Ubuntu system administration and ROS2 workspace management
- colcon build system and ament_cmake package structure

---

## 🚀 Roadmap

See [`Docs/PHASES.md`](Docs/PHASES.md) for the full detailed phase breakdown.

**Current focus:** Stabilising the Pi 4B ↔ ESP32 micro-ROS bridge on the real hardware so `/cmd_vel` commands reliably reach the wheels end-to-end before moving to SLAM and Nav2.

---

## 👤 Author

**Mayank Jain**
Robotics and Automation Engineer 

> *ROS2 architecture, ros2_control hardware interface plugin, URDF modelling, Gazebo simulation, micro-ROS ESP32 firmware, physical hardware build and wiring — all designed and built from scratch.*

[![GitHub](https://img.shields.io/badge/GitHub-Profile-black?logo=github)](https://github.com/MayankJain-22)

---

## 📄 License

Apache 2.0 — see [LICENSE](LICENSE)
