# Transcend Rover — Development Phases

This document tracks build progress. Updated as each milestone is reached.

---

## ✅ Phase 1 — Hardware Foundation
**Status: Complete**

- [x] 4WD chassis built with real measured dimensions
- [x] N20 encoder motors mounted — 4 wheels with quadrature encoders
- [x] TB6612FNG H-bridge motor drivers wired (front + rear pair)
- [x] Custom hardware mount built from scratch — camera, LiDAR, and Raspberry Pi all housed
- [x] Filter circuit + power management for 5V and 3.3V rails
- [x] Li-Po battery power system

---

## ✅ Phase 2 — ROS2 Workspace & Robot Description
**Status: Complete**

- [x] ROS2 Jazzy workspace set up on Raspberry Pi 4B (Ubuntu 24.04)
- [x] `transcend_description` package — full URDF/xacro
  - base_link, base_footprint, 4 wheel links
  - Correct inertia tensors calculated from real dimensions
  - `laser_frame` LiDAR mount link
  - Gazebo differential drive plugin
  - Joint state publisher plugin
- [x] `common_properties.xacro` — reusable inertia macros

---

## ✅ Phase 3 — ros2_control Hardware Interface
**Status: Complete**

- [x] `transcend_hardware` package — custom C++ `SystemInterface` plugin
- [x] RT-safe design: `write()` deposits into `pending_cmd_` — DDS publish stays off the RT thread
- [x] Background spin thread handles subscriber callbacks and publisher
- [x] `on_activate()` / `on_deactivate()` lifecycle hooks implemented
- [x] Custom `WheelVelocity.msg` — `/wheel_vel_cmd` and `/wheel_vel_state`
- [x] `diff_drive_controller` receiving `/cmd_vel` → 4 wheel velocity targets in rad/s
- [x] `joint_state_broadcaster` publishing encoder feedback
- [x] `use_stamped_vel: false` — standard `Twist` compatible with Nav2 and teleop

---

## ✅ Phase 4 — Bringup & Sensor Integration
**Status: Complete**

- [x] `transcend_bringup` — single `rover_pi.launch.xml` starts full stack
  - robot_state_publisher
  - ros2_control controller manager
  - joint_state_broadcaster + diff_drive_controller
  - YDLidar X2 driver (configured at `/dev/ttyUSB0`, 115200 baud)
  - micro_ros_agent (UDP port 8888 — ESP32 bridge)
- [x] Gazebo bridge config — `/cmd_vel` and `/joint_states` bridged ROS↔Gazebo
- [x] RViz2 config saved — `transcend_config.rviz`

---

## ✅ Phase 5 — Gazebo Harmonic Simulation
**Status: Complete**

- [x] Gazebo Harmonic differential drive simulation working
- [x] LiDAR scan simulation publishing on `/scan`
- [x] Joint state feedback from Gazebo into ROS2
- [x] Robot model fully visible and moving in Gazebo
- [x] ROS↔Gazebo bridge verified

---

## 🔄 Phase 6 — Real Hardware Integration
**Status: In Progress**

Getting the full stack running on the physical rover — Pi 4B + ESP32 + YDLidar all talking to each other reliably.

- [x] ESP32 micro-ROS firmware written and flashed
  - Full ROS2 node (`transcend_esp32`) running on microcontroller over WiFi UDP
  - Direction-aware quadrature encoder ISRs (signed tick counting)
  - PWM deadband (30/255) to overcome static friction at low speeds
  - PID integrator reset on direction reversal
  - 2s watchdog — motors stop if `/wheel_vel_cmd` stops arriving
  - Agent state machine: `WAITING_WIFI → WAITING_AGENT → AGENT_CONNECTED`
  - Bug fixes: executor spin window 1ms → 15ms, agent ping non-blocking (100ms, 1 attempt)
- [x] 98% straight-line motion achieved under PID control
- [ ] micro-ROS UDP bridge stable between Pi 4B and ESP32 under full ROS2 load
- [ ] YDLidar X2 stable on Pi 4B at `/dev/ttyUSB0`
- [ ] Full launch — all nodes running simultaneously on real rover
- [ ] `/cmd_vel` from teleop keyboard controlling real wheels end-to-end

---

## 🔲 Phase 7 — SLAM + Nav2 Navigation
**Status: Planned**

- [ ] SLAM Toolbox — building map from real LiDAR data
- [ ] Nav2 bringup — global + local costmap planners
- [ ] Autonomous waypoint navigation
- [ ] Obstacle avoidance in real environment

---

## 🔲 Phase 8 — Transcend Behaviour Layer
**Status: Planned**

The core intelligence that gives the rover its purpose.

- [ ] Schedule enforcement node — ROS2 Action server
- [ ] Doom-scroll detection — phone screen time API integration
- [ ] Physical interruption behaviour — rover autonomously drives to user
- [ ] Sleep/wake routine assistant
- [ ] Voice feedback module

---

## 🔲 Phase 9 — AI/ML Integration
**Status: Planned**

- [ ] Person detection and tracking — camera + ML model
- [ ] Room-level localisation — knowing where the user is
- [ ] Behaviour pattern learning — adapting to user's routine
- [ ] Depth estimation from monocular camera

---

## 🔲 Phase 10 — Full System Integration
**Status: Planned**

- [ ] All hardware subsystems running reliably together
- [ ] Full autonomous loop: detect → navigate → interrupt → return to dock
- [ ] Mobile companion app
- [ ] Cloud logging and analytics dashboard
- [ ] Power-optimised operation for all-day use
