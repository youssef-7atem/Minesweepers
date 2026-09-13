# Landmine-Detection Rover
<img width="513" height="296" alt="rover_render" src="https://github.com/user-attachments/assets/7aded1e0-9f23-4f76-9db7-b3977c86a7df" />
![Uploading sensor_architecture_diagram.svg…]()
<img width="5604" height="1963" alt="sensor_architecture_diagram" src="https://github.com/user-attachments/assets/a4082333-bad6-4f56-83d9-96d1c901f8ef" />


An autonomous/teleoperated 4-wheel-drive rover that searches for buried landmines using a metal-detecting coil and a camera-based deep-learning classifier, marks their location on a grid map, and retrieves them with an electromagnetic gripper arm.

<p align="center">
  <img src="docs/rover_render.png" width="650" alt="Rover CAD render">
</p>

---

## Table of Contents

- [Overview](#overview)
- [Mechanical Design](#mechanical-design)
- [Electrical & Sensor Architecture](#electrical--sensor-architecture)
- [Compute & Communication](#compute--communication)
- [Software / ROS Architecture](#software--ros-architecture)
- [Detection: Coil & Camera](#detection-coil--camera)
- [Control Algorithm & Localization](#control-algorithm--localization)
- [Mapping](#mapping)
- [Repository Structure](#repository-structure)
- [Team](#team)

---

## Overview

The rover is a 4-wheeled, skid/differential-style vehicle built to:

1. **Drive** over rough, mine-field-like terrain using large, slightly-deflated, air-filled tires.
2. **Detect** buried metallic mines with an underslung metal-detecting coil and confirm/classify targets visually with a YOLOv8 model.
3. **Localize itself** using wheel-encoder odometry fused with IMU yaw data through an Extended Kalman Filter (EKF), with a PID controller holding heading.
4. **Mark** each detected mine on a grid map of the field.
5. **Retrieve** the mine with an electromagnetic gripper arm and carry it in an onboard collection box.
6. **Alert** operators audibly (siren/buzzer) on detection, and stream live video back to a ground control station over Wi-Fi.

---

## Mechanical Design

### Chassis
- **Aluminum extrusions** — high strength-to-weight ratio, non-magnetic (won't interfere with the metal detector).
- **6 mm MDF wood** panels for structural/mounting surfaces.
- **4 mm aluminum double-membrane rods** with spacers to minimize weight.
- **Thin metal sheet cover** to protect internal electronics from the environment.

### Locomotion System
- **4× DC motors, 110 RPM** — high torque, improved traction, simple to control and power-efficient.
- **Air-filled tires**, run slightly deflated for adaptive suspension (absorbs shocks on rough terrain) and a larger footprint for traction on soft/sandy ground.
- **Independent encoder-equipped tires** for accurate odometry.

### Sensor Fixation (Metal-Detector Coil)
- Held out in front of the chassis on **2 PVC pipes + a thin MDF rectangle**.
- Both materials are **non-magnetic**, chosen specifically so nearby metallic robot components don't create false readings on the coil.

### Gripper Arm
- **High-torque motor** driven arm.
- **End effector fitted with electromagnets** to lift ferrous mine casings.
- **Onboard box** to contain collected mines.

---

## Electrical & Sensor Architecture

Full power, control, and sensor wiring diagram:

<p align="center">
  <img src="docs/sensor_architecture_diagram.png" width="900" alt="Sensor and electrical architecture diagram">
</p>

*(Vector version: [`docs/sensor_architecture_diagram.svg`](docs/sensor_architecture_diagram.svg))*

### Power Distribution
| Rail | Source | Feeds |
|---|---|---|
| 12 V | 2× 12 V 35 Ah sealed lead-acid batteries (shared bus) | Drive-train MDD10A drivers, relay module, buck converter |
| 5 V | Buck converter (stepped down from 12 V) | Raspberry Pi logic / Arduino logic where needed |
| 3.7 V | Dedicated LiPo (1000 mAh) | Raspberry Pi |
| 3.7 V | Dedicated LiPo (2000 mAh) | Gripper-arm motor driver |

### Core Components
- **Raspberry Pi** — hosts ROS master/nodes, runs detection (YOLOv8) and EKF localization, handles the Wi-Fi link to the ground station.
- **Arduino Uno** — low-level control: motor PWM/direction to the motor drivers, wheel-encoder pulse counting, metal-detector coil ADC reading, relay control signal.
- **3× MDD10A dual-channel 10 A DC motor drivers** — drive the 4 wheel motors and the gripper motor.
- **IMU (MPU6050/9250, GY-91 breakout)** — yaw/orientation data over I2C, fused into the EKF.
- **Wheel encoders** — per-wheel tick counters for odometry.
- **Metal-detecting coil** — analog front-end read by the Arduino.
- **Pi Camera** — video stream for the mine-classification model.
- **2-channel relay module** — switches the electromagnet and the siren/buzzer.
- **Electromagnet** — mounted on the gripper end effector.
- **Siren & buzzer** — audible alert on mine detection.
- **Joystick + laptop** — ground control station for teleoperation and monitoring.

---

## Compute & Communication

| Side | Hardware |
|---|---|
| **Ground Station** | Laptop (ROS master / GUI / video receiver), Joystick controller |
| **Robot** | Raspberry Pi, Arduino Uno, Pi Camera |

The robot and ground station communicate over **Wi-Fi**, running **ROS** across both machines: joystick teleop commands flow from the laptop to the robot, while odometry, detection, and video topics flow back.

---

## Software / ROS Architecture

Node graph (from `rqt_graph`):

```
/joystick → /controller → /arduino → /left_ticks  ─┐
                        ↘ /move_robot   /right_ticks ┼→ /encoder → /odom ─┐
n___mpu6050_node → /imu → /imu/data ──────────────────────────────────────┼→ /ekf → /mine_pose → /map
                                        /camera → /mine_theta ────────────┘
                        ↘ /arduino → /detection ────────────────────────→ /ekf
```

- **`/controller`** converts joystick input into robot velocity commands (`/move_robot`) sent to the Arduino.
- **`/arduino`** publishes raw encoder ticks (`/left_ticks`, `/right_ticks`) and coil detections (`/detection`).
- **`/encoder`** converts wheel ticks into `/odom` (position estimate from wheel kinematics).
- **`n___mpu6050_node`** publishes IMU orientation on `/imu/data`.
- **`/camera`** publishes the visually-estimated mine bearing (`/mine_theta`).
- **`/ekf`** fuses odometry + IMU + detection/vision cues into a filtered pose, published as `/mine_pose`.
- **`/map`** consumes the fused pose stream to place detected mines on the grid map.

### Robot Kinematics & Odometry

```math
d_{l,r} = (T_t - T_{t-1}) / N
```

```math
P_R = \begin{bmatrix} x \\ y \\ \theta \end{bmatrix} + \begin{bmatrix} d\cos(\theta) \\ d\sin(\theta) \\ \Delta\theta \end{bmatrix}
```

Where `d_l,r` is the distance traveled by each wheel between the last two encoder samples (ticks `T` over `N` ticks/revolution), and `P_R` is the updated robot pose.

### Sensor Fusion — Extended Kalman Filter (EKF)

State estimated: `x, y, θ` (position + heading). Odometry provides the prediction step; IMU yaw provides the measurement update on `θ`.

**Prediction**
```math
\hat{x}_k = f(x_{k-1}, u) \quad \text{(non-linear)}
```
```math
P_k = A_x P_{k-1} A_x^T + B_u Q B_u^T
```

**Update**
```math
G_k = P_k C_x^T (C_x P_k C_x^T + R)^{-1}
```
```math
\hat{x}_k \leftarrow \hat{x}_k + G_k (z_k - h(\hat{x}_k))
```
```math
P_k \leftarrow (I - G_k C_x) P_k
```

### PID Heading Control
A PID loop closes on yaw angle (`τ(t)` setpoint vs. measured `y(t)`), correcting motor commands to hold a commanded heading:

```math
u(t) = K_p e(t) + K_i \int_0^t e(t)\,dt + K_d \frac{de(t)}{dt}
```

---

## Detection: Coil & Camera

- **Deep learning pipeline (YOLOv8):** custom dataset — data collection → labelling → model training — reaching **~90% detection accuracy** on the target classes.
- **Video stream:** low-latency RTSP (**~100 ms**) using **GStreamer**, processed with **OpenCV** on the receiving end.
- The metal-detector coil gives the first-pass "something's there" signal; the camera/YOLOv8 model confirms and classifies the object before it's logged as a mine.

---

## Mapping

Detected mines (red) and reference/marker points (blue) are placed on a discretized grid (`0–J` × `0–11`) built from the fused `/mine_pose` estimates, giving the operator a top-down record of the swept field.

---

## Repository Structure

Suggested layout — adjust to match your actual repo:

```
.
├── docs/
│   ├── sensor_architecture_diagram.png
│   ├── sensor_architecture_diagram.svg
│   └── rover_render.png
├── arduino/            # Arduino Uno firmware (motor PWM, encoders, coil ADC)
├── ros_ws/             # ROS workspace (nodes: controller, encoder, ekf, detection, map)
├── vision/             # YOLOv8 training/inference code, dataset notes
├── cad/                # Mechanical design files
└── README.md
```

---

## Team

Built by the **E-JUST Robotics Club**.

<p align="center">
  <img src="docs/rover_render.png" width="120" alt="">
</p>
