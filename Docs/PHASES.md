# Transcend Rover — Development Phases

This document tracks the build progress of the Transcend rover.
The project is actively being developed and phases are updated as milestones are completed.

---

## ✅ Phase 1 — Hardware Foundation & Motion Control
**Status: Complete**

- [x] 4WD chassis built — real rover physical dimensions measured and modelled
- [x] N20 encoder motors mounted — 4 wheels with quadrature encoders
- [x] ESP32 firmware — PID speed control per wheel, micro-ROS UDP bridge
- [x] 98% straight-line motion achieved under PID control
- [x] Filter circuit + power management for ESP32 and Raspberry Pi
- [x] Custom hardware mount built for camera, LiDAR, and Raspberry Pi

---

## ✅ Phase 2 — ROS2 Workspace & Robot Description
**Status: Complete**

- [x] ROS2 Jazzy workspace set up on Raspberry Pi 5 (Ubuntu 24.04)
- [x] `transcend_description` package — full URDF/xacro with real dimensions
  - base_link, base_footprint, 4 wheel links with correct inertia tensors
  - LiDAR mount link (`laser_frame`) on top
  - Gazebo differential drive plugin
  - Joint state publisher plugin
- [x] `common_properties.xacro` — reusable inertia macros (box, cylinder, sphere)

---

## ✅ Phase 3 — ros2_control Hardware Interface
**Status: Complete**

- [x] `transcend_hardware` package — custom `SystemInterface` plugin
- [x] RT-safe design: `write()` deposits into `pending_cmd_` — DDS publish is off the RT thread
- [x] Background spin thread handles both subscriber callbacks and publisher
- [x] `on_activate()` / `on_deactivate()` lifecycle hooks
- [x] `WheelVelocity.msg` — custom message type for `/wheel_vel_cmd` and `/wheel_vel_state`
- [x] `diff_drive_controller` receiving `/cmd_vel` → splitting to 4 wheel velocity targets
- [x] `joint_state_broadcaster` publishing real encoder feedback
- [x] `use_stamped_vel: false` — accepts standard `Twist` (compatible with Nav2 and teleop)

---

## ✅ Phase 4 — Bringup & Sensor Integration
**Status: Complete**

- [x] `transcend_bringup` package — single `rover_pi.launch.xml` brings up full stack
  - robot_state_publisher
  - ros2_control controller manager
  - joint_state_broadcaster + diff_drive_controller
  - YDLidar X2 driver node (configured at `/dev/ttyUSB0`, 115200 baud)
  - micro_ros_agent (UDP port 8888 — ESP32 bridge)
- [x] Gazebo bridge config — `/cmd_vel` and `/joint_states` bridged ROS↔Gazebo
- [x] RViz2 config saved — `transcend_config.rviz`

---

## 🔄 Phase 5 — Gazebo Simulation
**Status: In Progress**

- [x] Gazebo Harmonic differential drive simulation working
- [ ] Full sensor simulation — LiDAR scan in Gazebo
- [ ] Camera simulation with visual topic
- [ ] Hardware-in-the-loop testing (Pi ↔ Gazebo)

---

## 🔲 Phase 6 — Navigation (Nav2)
**Status: Planned**

- [ ] SLAM Toolbox integration — building map from LiDAR
- [ ] Nav2 bringup — global + local planner
- [ ] Autonomous waypoint navigation
- [ ] Obstacle avoidance

---

## 🔲 Phase 7 — Behaviour Layer (Transcend Logic)
**Status: Planned**

This is the core intelligence layer that gives the rover its purpose.

- [ ] Schedule enforcement node — ROS2 Action server
- [ ] Doom-scroll detection — phone screen time API integration
- [ ] Physical interruption behaviour — rover drives to user location
- [ ] Sleep/wake routine assistant
- [ ] Person tracking (camera + depth estimation)
- [ ] Voice feedback module

---

## 🔲 Phase 8 — Full System Integration
**Status: Planned**

- [ ] Raspberry Pi 5 + ESP32 + LiDAR fully integrated on real rover
- [ ] Full autonomous loop: detect → navigate → interrupt → return
- [ ] Mobile companion app
- [ ] Cloud logging and analytics dashboard
