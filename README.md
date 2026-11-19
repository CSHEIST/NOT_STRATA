# ROS 2 Waypoint-Navigated Rover with a Vision-Guided Robotic Arm

A modular robotics project that combines a mobile rover, a tabletop robotic arm, and a camera-based sorting workstation. The intended system uses **ROS 2 waypoint navigation** to bring the rover to a workstation, detects coloured objects with a fixed camera, and uses the robotic arm to sort them.

> **Development status:** This repository documents an incremental project. The current arm-control milestone is **manual RViz joint-slider control through ROS 2, UDP, an ESP32, and a PCA9685 servo driver**. The elbow joint and a separate virtual gripper command slider are included in the robot description/control path. IK, validated vision-to-workspace calibration, and fully autonomous pick-and-place are future integration milestones; they should not be considered complete until tested on the physical system.

---

## Contents

- [Project Overview](#project-overview)
- [System Architecture](#system-architecture)
- [Hardware](#hardware)
- [Software Stack](#software-stack)
- [Repository Layout](#repository-layout)
- [Current Implementation Status](#current-implementation-status)
- [Getting Started](#getting-started)
- [Manual Arm Control](#manual-arm-control)
- [ROS 2 to ESP32 Communication](#ros-2-to-esp32-communication)
- [Calibration and Safety](#calibration-and-safety)
- [Planned Development](#planned-development)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)

---

## Project Overview

The project aims to develop a mobile manipulation platform that can travel to a defined workstation, identify small coloured objects, and sort them using a robotic arm.

The intended workflow is:

1. The rover follows a sequence of navigation waypoints to reach the sorting workstation.
2. The rover reports that it has reached the workstation and remains stationary during manipulation.
3. A fixed USB camera captures the workspace.
4. An OpenCV-based vision pipeline identifies the target object's colour and estimates its image centre.
5. A camera-to-workspace calibration maps image coordinates into workspace coordinates, followed by a transform into the robot's base frame.
6. The arm is commanded to approach, grasp, and move the object to its designated location.
7. After the sorting sequence has completed, the rover can be instructed to return to its starting location.

The immediate implementation priority is to establish reliable and safe **manual joint actuation** before adding inverse kinematics or autonomous manipulation.

## System Architecture

### Intended end-to-end architecture

```text
                    ROS 2 workstation / laptop
┌─────────────────────────────────────────────────────────────────┐
│                                                                 │
│  Waypoint Navigation       Robot Description / RViz             │
│  (rover)                   Joint State Publisher GUI             │
│       │                              │                          │
│       ▼                              ▼                          │
│  Rover controller             /joint_states                     │
│                                     │                           │
│                                     ▼                           │
│                            Python UDP bridge                     │
│                                     │                           │
│                                     ▼                           │
│                              Wi-Fi / UDP                         │
│                                     │                           │
└─────────────────────────────────────┼───────────────────────────┘
                                      ▼
                              ESP32 access point
                                      │
                                      ▼
                               PCA9685 PWM driver
                                      │
                         ┌────────────┴────────────┐
                         ▼                         ▼
                   Arm servos                Gripper servo

  USB camera ──> OpenCV colour detection ──> workspace calibration
                                               │
                                               ▼
                                     Future IK / arm targets
```

The diagram describes the **target architecture**. Not every connection in the full perception-to-manipulation pipeline has been validated yet.

### Arm-control data flow currently being developed

```text
RViz joint sliders
       │
       ▼
Joint State Publisher GUI
       │  sensor_msgs/msg/JointState
       ▼
ROS 2 topic: /joint_states
       │
       ▼
arm_esp32_bridge.py
       │  UDP datagrams
       ▼
ESP32 Wi-Fi access point
       │
       ▼
PCA9685 PWM servo driver
       │
       ▼
Physical servos
```

## Hardware

The project architecture uses the following hardware categories. Confirm exact board, servo, channel, and power-supply models against the physical build before using the firmware.

| Subsystem | Component | Role |
|---|---|---|
| Mobile platform | Rover chassis and drive motors | Moves between the start point and workstation |
| Rover control | Rover microcontroller and motor driver | Executes low-level drive commands |
| Manipulator | Multi-joint servo-driven robotic arm | Performs object manipulation |
| Arm controller | ESP32 | Receives joint commands over Wi-Fi UDP |
| Servo driver | PCA9685 16-channel PWM controller | Generates PWM signals for the servos |
| Vision | Fixed USB camera | Captures the sorting workspace |
| Processing | Laptop / computer | Runs ROS 2, RViz, the bridge, and planned vision software |
| Work surface | Marked tabletop workspace | Provides a reference area for object localization |
| Power | External regulated servo supply | Powers the servos independently of the ESP32 logic supply |

The workspace design has been discussed around a nominal **400 mm × 250 mm usable area**, with reference markers intended near its corners. The intended object set is small coloured cubes (red, blue, and yellow). Measure the actual board and marker positions before using these dimensions for calibration.

## Software Stack

| Software / package | Purpose |
|---|---|
| ROS 2 | Communication and application framework |
| `robotic_arm_description` | URDF/Xacro robot model, meshes, launch and RViz configuration |
| RViz | Visualizes the robot model and joint motion |
| `joint_state_publisher_gui` | Provides manual joint sliders |
| `rclpy` | Python ROS 2 client library used by the bridge |
| `sensor_msgs` | Provides the `JointState` message type |
| Python UDP socket | Sends joint commands from the laptop to the ESP32 |
| ESP32 Arduino core | Runs the embedded UDP receiver |
| Adafruit PWM Servo Driver library | Controls the PCA9685 |
| OpenCV | Planned colour segmentation and object-centre estimation |
| MoveIt 2 / KDL | Planned inverse-kinematics and motion-planning stage |
| Nav2 / waypoint navigation | Rover navigation subsystem, developed separately |

The saved arm-package instructions use ROS 2 **Jazzy** in their example commands. Replace `jazzy` with the ROS 2 distribution installed on your computer if it differs.

## Repository Layout

A suggested repository structure is shown below. Keep the existing package files and mesh paths intact when organizing the repository; update paths and launch references only when necessary.

```text
ROS2-Waypoint-Rover-Vision-Guided-Robotic-Arm/
│
├── robotic_arm_description/
│   ├── urdf/
│   │   └── robotic_arm_fixed_gripper.urdf.xacro
│   ├── meshes/
│   ├── launch/
│   ├── rviz/
│   ├── scripts/
│   │   └── arm_esp32_bridge.py
│   ├── package.xml
│   └── setup.py
│
├── arm_hardware/
│   └── firmware/
│       └── arm_esp32_bridge.ino
│
├── waypoint_navigation/
│   └── ...
│
├── computer_vision/
│   └── ...
│
├── documentation/
│   ├── wiring/
│   ├── calibration/
│   └── test_logs/
│
└── README.md
```

`waypoint_navigation/` and `computer_vision/` are organizational locations for their respective work. Add their source files as those subsystems are integrated; do not treat the placeholder directories as implemented packages.

## Current Implementation Status

| Subsystem | Current status |
|---|---|
| ROS 2 arm description package | Package structure and robot-description assets exist; verify the active launch file and link/joint tree |
| RViz model and manual sliders | Default Joint State Publisher GUI is the selected manual interface |
| Base and shoulder mapping | Preliminary software mappings are documented; physical calibration and servo-type verification remain necessary |
| Elbow slider | `elbow_joint` is defined in the saved Xacro; bridge/firmware support is included in the current control bundle and requires hardware validation |
| Wrist slider | Joint is defined; physical calibration and validation remain pending |
| Virtual gripper slider | `gripper_command_joint` is a separate virtual slider; it is intended to send discrete OPEN/CLOSE commands without animating the gripper mesh in RViz |
| ROS 2 to ESP32 UDP path | A coordinated bridge and receiver implementation is provided; verify live packet reception before connecting servo power |
| Rover waypoint navigation | Developed separately; complete workstation handoff and return sequence require validation |
| Colour detection | Planned with OpenCV; detection performance has not yet been validated |
| Camera/workspace calibration | Marker-based calibration approach planned; measure and validate transformation error |
| MoveIt 2 / IK | Not yet configured and validated for the physical arm |
| Autonomous pick-and-place | Future integration milestone |

**Status rule:** A file existing in the repository does not prove that its functionality has been tested on the physical robot. Add screenshots, logs, measurements, and videos when each milestone is verified.

## Getting Started

### 1. Prerequisites

- Ubuntu with a compatible ROS 2 installation.
- `colcon` build tools and the ROS 2 development environment.
- The existing `robotic_arm_description` package and its required meshes/assets.
- ESP32 Arduino support in the Arduino IDE or another compatible build environment.
- Adafruit PWM Servo Driver library installed in the Arduino environment.
- ESP32 and laptop able to communicate over the same configured Wi-Fi network.
- PCA9685 and servo wiring checked against the actual hardware.

### 2. Place the files

Use the existing ROS 2 workspace and package. The project notes refer to `~/robotic_arm_ws` and package `robotic_arm_description`.

Place the robot description at the path used by the existing launch file. If that launch file already references another Xacro filename, update that file rather than creating two competing robot descriptions.

Place the Python bridge at:

```text
~/robotic_arm_ws/src/robotic_arm_description/scripts/arm_esp32_bridge.py
```

Place the ESP32 sketch in your firmware folder and open it in the Arduino IDE. Do not flash it until the Wi-Fi credentials, PCA9685 channels, servo types, and safe pulse ranges have been checked.

### 3. Check package dependencies and script installation

Keep the existing `setup.py` and `package.xml`; do not replace them wholesale. Confirm that:

- `setup.py` installs `scripts/arm_esp32_bridge.py` using the package's existing script or entry-point convention.
- `package.xml` declares runtime dependencies on `rclpy` and `sensor_msgs` if they are not already present.
- The bridge script is executable.

For example, the runtime dependencies are:

```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>sensor_msgs</exec_depend>
```

### 4. Build the arm package

```bash
source /opt/ros/jazzy/setup.bash
cd ~/robotic_arm_ws
colcon build --packages-select robotic_arm_description
source install/setup.bash
```

### 5. Launch the existing RViz model

Use the launch file already configured in your package. Its exact filename is installation-specific, so use the launch command that currently opens your robot model and Joint State Publisher GUI.

Move each slider and confirm that `/joint_states` is publishing the expected joint names before running the bridge.

### 6. Check the joint-state topic

In a new terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/robotic_arm_ws/install/setup.bash
ros2 topic echo /joint_states
```

The expected joint names for the current model are:

```text
base_joint
shoulder_joint
elbow_joint
wrist_joint
gripper_command_joint
```

Confirm that the position value changes when each corresponding slider is moved.

### 7. Run the bridge

In another terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/robotic_arm_ws/install/setup.bash
ros2 run robotic_arm_description arm_esp32_bridge.py
```

If ROS 2 cannot find the executable, check the existing `setup.py` installation configuration, rebuild the package, and source `install/setup.bash` again.

## Manual Arm Control

The current manual control stage uses the default Joint State Publisher GUI in RViz.

| Slider / joint | Intended role |
|---|---|
| `base_joint` | Base rotation |
| `shoulder_joint` | Shoulder movement |
| `elbow_joint` | Elbow movement |
| `wrist_joint` | Wrist movement |
| `gripper_command_joint` | Virtual OPEN/CLOSE command; it is not the physical gripper joint |

The gripper slider is intentionally separate from the physical gripper mount. Moving it does not animate the gripper mesh in RViz; instead, it is interpreted as a discrete command by the bridge.

- Slider value greater than `+0.1 rad`: send `GRIPPER:OPEN`.
- Slider value less than `-0.1 rad`: send `GRIPPER:CLOSE`.
- Slider near zero: send no new gripper command; the physical gripper retains its last commanded position.

Move sliders slowly and test with the arm unloaded. Do not move a joint through the entire virtual range until its physical direction and safe limits have been established.

## ROS 2 to ESP32 Communication

### Network configuration

The development configuration recorded for the bridge is:

| Setting | Recorded value |
|---|---|
| ESP32 access-point IP | `192.168.4.1` |
| Laptop IP | `192.168.4.2` |
| UDP port | `4210` |
| ROS 2 topic | `/joint_states` |

These are configuration values from the development setup, not values guaranteed for every network. Confirm the live addresses and port before testing.

### Joint packet format

The Python bridge sends a joint packet in this format:

```text
BASE=90.0;SHOULDER=94.5;ELBOW_URDF=30.0;WRIST_URDF=0.0
```

The gripper uses separate UDP datagrams:

```text
GRIPPER:OPEN
GRIPPER:CLOSE
```

### Current software mappings

| Axis | Input and mapping |
|---|---|
| Base | URDF range approximately `-73.2813°` to `+73.2813°` is mapped to a preliminary servo command range of `10°` to `170°` |
| Shoulder | URDF range `-90°` to `+90°` is mapped to a preliminary servo command range of `17°` to `172°` |
| Elbow | URDF joint angle in degrees is sent as `ELBOW_URDF`; firmware maps `-90°…+90°` to `0°…180°` by adding `90°` |
| Wrist | URDF joint angle in degrees is sent as `WRIST_URDF`; firmware uses the same `+90°` offset |
| Gripper | Separate OPEN/CLOSE command; intended servo targets are `60°` and `90°` respectively |

These are **initial software conventions**, not measured calibration results. Servo direction, neutral position, pulse-width limits, mechanical travel, and PCA9685 channel assignments must be verified on the actual hardware.

### Default PCA9685 channels in the example firmware

| Servo | Default channel in example code |
|---|---:|
| Base | 0 |
| Shoulder | 1 |
| Elbow | 2 |
| Wrist | 3 |
| Gripper | 4 |

The channel numbers are placeholders until checked against the wiring. Update the firmware constants to match the actual connections.

## Calibration and Safety

**Do not power the servos or test full-range motion until the following checks are complete.**

1. Confirm the exact servo model and whether each actuator is a positional servo or a continuous-rotation servo.
2. A continuous-rotation servo interprets its signal as speed/direction rather than an absolute shaft angle. The Base position mapping in the example bridge is not valid for a continuous-rotation servo.
3. Confirm ESP32 access-point credentials, laptop IP, ESP32 IP, and UDP port.
4. Confirm PCA9685 I²C address and every servo channel against the physical wiring.
5. Use a suitable external regulated servo power supply. Do not power multiple servos from the ESP32 3.3 V pin. Connect grounds appropriately between the ESP32, PCA9685, and external supply.
6. First verify that the Python bridge sends packets and that the ESP32 serial monitor receives and parses them with servo power disconnected.
7. Begin physical testing with the arm unloaded, at low risk, and with an immediate power-disconnect available.
8. Calibrate pulse-width range and mechanical endpoints for each servo individually. Do not assume `500–2500 µs` is safe for every servo.
9. Stop testing if a servo hits a hard stop, stalls, hums continuously, overheats, or the power supply voltage drops.
10. Record joint direction, safe angle range, neutral angle, PWM pulse range, channel number, and test outcome before enabling combined movement.

The saved Xacro contains temporary or unusually large origins in parts of the robot model. Validate link geometry, joint axes, mesh scales, and physical mounting against the real arm before using the model for IK or autonomous motion.

## Planned Development

The project is being developed in milestones:

1. **Manual actuator interface:** Verify `/joint_states`, Python bridge output, ESP32 packet reception, and individual Base/Shoulder/Elbow/Wrist servo response.
2. **Gripper command control:** Verify the virtual gripper slider and OPEN/CLOSE commands with calibrated physical gripper positions.
3. **Robot model validation:** Reconcile URDF/Xacro joint origins, axes, limits, mesh scales, and mounting geometry with the physical manipulator.
4. **Rover waypoint integration:** Complete waypoint arrival reporting, workstation hold, manipulation handoff, and return-to-start sequence.
5. **Vision pipeline:** Implement colour segmentation, contour filtering, object-centre estimation, and annotated camera output.
6. **Workspace calibration:** Use measured reference markers to map image coordinates to the workspace and robot base frame; quantify coordinate error.
7. **Inverse kinematics:** Configure MoveIt 2 and a suitable solver only after the robot description is geometrically validated; test reachable targets and verify forward-kinematics error.
8. **Autonomous manipulation:** Integrate approach, grasp, lift, sorting, placement verification, recovery, and rover return into a supervised state machine.
9. **Performance evaluation:** Record navigation endpoint error, detection accuracy, coordinate error, IK success rate, repeatability, grasp success rate, and cycle time.

## Troubleshooting

| Symptom | Checks |
|---|---|
| `/joint_states` does not appear | Confirm RViz launch includes Joint State Publisher GUI and that the correct ROS environment is sourced |
| Joint names are missing | Compare the names in the active Xacro with the expected names in this README and in the bridge |
| Bridge starts but sends no packets | Check the topic with `ros2 topic echo /joint_states`; the bridge waits until Base, Shoulder, and Elbow states have been received |
| `ros2 run` cannot find the bridge | Check script installation in the existing `setup.py`, executable permission, rebuild, and source the workspace overlay |
| ESP32 receives no UDP data | Verify Wi-Fi association, IP addresses, UDP port `4210`, firewall rules, and ESP32 serial output |
| Wrong servo moves | Verify PCA9685 channel constants against the wiring |
| Servo moves in the wrong direction | Calibrate direction and update the mapping only after confirming the safe mechanical range |
| Servo jitters or stalls | Check power supply capacity, common ground, pulse limits, mechanical binding, and servo type |
| Gripper opens/closes unexpectedly | Verify `gripper_command_joint` is present and test the `+0.1 rad` / `-0.1 rad` thresholds with servo power disconnected first |

## Contributing

Contributions and suggestions are welcome. When submitting changes:

- Keep ROS 2 package dependencies and launch configuration consistent.
- Do not change joint names or UDP field names without updating both the Python bridge and ESP32 firmware.
- Document servo channel changes and calibration values.
- Separate simulated results from measured physical results.
- Include reproducible test steps, logs, and photographs or videos for hardware changes.

## License

No license has been selected for this repository yet. Add a `LICENSE` file before permitting reuse, redistribution, or contributions under a specific license.

---

## Project Note

This repository is an evolving academic robotics project. Features described as planned or in progress should not be interpreted as fully implemented or experimentally validated. Hardware-specific settings must be confirmed on the actual rover and robotic arm before operation.
