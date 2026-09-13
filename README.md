# Landmine-Detection Rover
<img width="513" height="296" alt="rover_render" src="https://github.com/user-attachments/assets/7aded1e0-9f23-4f76-9db7-b3977c86a7df" />
<img width="2691" height="943" alt="sensor_architecture_diagram" src="https://github.com/user-attachments/assets/7e3be5a1-258e-4015-aa21-d43e3c1badda" />
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
![Uploading sensor_architecture_diagram.svg…]()
<?xml version="1.0" encoding="UTF-8" standalone="no"?>
<!DOCTYPE svg PUBLIC "-//W3C//DTD SVG 1.1//EN"
 "http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd">
<!-- Generated by graphviz version 2.43.0 (0)
 -->
<!-- Title: MinesweeperRobot Pages: 1 -->
<svg width="2018pt" height="707pt"
 viewBox="0.00 0.00 2017.60 706.60" xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" xmlns:c2pa="http://c2pa.org/manifest"><metadata><c2pa:manifest>AAAWgmp1bWIAAAAeanVtZGMycGEAEQAQgAAAqgA4m3EDYzJwYQAAABZcanVtYgAAAEdqdW1kYzJtYQARABCAAACqADibcQN1cm46YzJwYTpjZTU4YmIzYS01ZTQ5LTRmMTktYTU4Yi1jZGI5OWE3ODc0ZTcAAAADl2p1bWIAAAApanVtZGMyYXMAEQAQgAAAqgA4m3EDYzJwYS5hc3NlcnRpb25zAAAAALxqdW1iAAAARGp1bWRjYm9yABEAEIAAAKoAOJtxE2MycGEuaW5ncmVkaWVudC52MwAAAAAYYzJzaBJYG9H4DK1ppQWKYx4R6qkAAABwY2JvcqNpZGM6Zm9ybWF0bWltYWdlL3N2Zyt4bWxqaW5zdGFuY2VJRHgseG1wOmlpZDpkMDg1ZTU0YS03OTkwLTQzM2MtYTQ0MC0yM2JlYzgyMTMxMTdscmVsYXRpb25zaGlwaHBhcmVudE9mAAAB4mp1bWIAAABBanVtZGNib3IAEQAQgAAAqgA4m3ETYzJwYS5hY3Rpb25zLnYyAAAAABhjMnNo0sbWGm051dv2wxbnkMJ7SAAAAZljYm9yomdhY3Rpb25zgqJmYWN0aW9ua2MycGEub3BlbmVkanBhcmFtZXRlcnOha2luZ3JlZGllbnRzgaJjdXJseC1zZWxmI2p1bWJmPWMycGEuYXNzZXJ0aW9ucy9jMnBhLmluZ3JlZGllbnQudjNkaGFzaFgg42W6oghC63olDGZapkK2frfYRlfH8mGl+JNzl5iPQBykZmFjdGlvbngdY29tLmFudGhyb3BpYy5jbGF1ZGUucHJvdmlkZWRqcGFyYW1ldGVyc6F4H2NvbS5hbnRocm9waWMub3JpZ2luLWNvbmZpZGVuY2VndW5rbm93bmtkZXNjcmlwdGlvbnhmQ2xhdWRlIHByb3ZpZGVkIHRoaXMgZmlsZSBhdCB0aGUgcmVxdWVzdCBvZiBhIHVzZXIgYW5kIG1heSBoYXZlIGNyZWF0ZWQgb3IgbW9kaWZpZWQgdGhlIGZpbGUgY29udGVudHMubXNvZnR3YXJlQWdlbnShZG5hbWVmQ2xhdWRlcmFsbEFjdGlvbnNJbmNsdWRlZPUAAADIanVtYgAAAEBqdW1kY2JvcgARABCAAACqADibcRNjMnBhLmhhc2guZGF0YQAAAAAYYzJzaBnrLclG3BP2jjQDJNQTOxAAAACAY2JvcqVjYWxnZnNoYTI1NmNwYWRMAAAAAAAAAAAAAAAAZGhhc2hYIMp00Y8BGG8B1+gpBivbhY2ox84ehM4uAMGETZRk8e8jZG5hbWVuanVtYmYgbWFuaWZlc3RqZXhjbHVzaW9uc4GiZXN0YXJ0GQHMZmxlbmd0aBkeBAAAAj5qdW1iAAAAJ2p1bWRjMmNsABEAEIAAAKoAOJtxA2MycGEuY2xhaW0udjIAAAACD2Nib3KlY2FsZ2ZzaGEyNTZpc2lnbmF0dXJleE1zZWxmI2p1bWJmPS9jMnBhL3VybjpjMnBhOmNlNThiYjNhLTVlNDktNGYxOS1hNThiLWNkYjk5YTc4NzRlNy9jMnBhLnNpZ25hdHVyZWppbnN0YW5jZUlEeCx4bXA6aWlkOmU3ZTE1YWZhLTkwNDQtNGNhNS1hMGMzLTJiZWM0ZmEwY2E4OXJjcmVhdGVkX2Fzc2VydGlvbnODomN1cmx4LXNlbGYjanVtYmY9YzJwYS5hc3NlcnRpb25zL2MycGEuaW5ncmVkaWVudC52M2RoYXNoWCDjZbqiCELreiUMZlqmQrZ+t9hGV8fyYaX4k3OXmI9AHKJjdXJseCpzZWxmI2p1bWJmPWMycGEuYXNzZXJ0aW9ucy9jMnBhLmFjdGlvbnMudjJkaGFzaFggWfZxh2EXZiDYNbEfQ/MLRJnYp7OhJlq3iF824WJsCUiiY3VybHgpc2VsZiNqdW1iZj1jMnBhLmFzc2VydGlvbnMvYzJwYS5oYXNoLmRhdGFkaGFzaFggcRgrD623mrZg3GhsOTSbSYPhl0ysUxXkjmL7g15Uv3h0Y2xhaW1fZ2VuZXJhdG9yX2luZm+jZG5hbWVvQW50aHJvcGljIEZpbGVzZ3ZlcnNpb25lMS4wLjBrc3BlY1ZlcnNpb25lMi40LjAAABA4anVtYgAAAChqdW1kYzJjcwARABCAAACqADibcQNjMnBhLnNpZ25hdHVyZQAAABAIY2JvctKEWQISogEmGCFZAgowggIGMIIBjaADAgECAhRA5aAK7sI50L64g/oGQgU9Z1UTADAKBggqhkjOPQQDAzBJMRcwFQYDVQQKEw5BbnRocm9waWMsIFBCQzEuMCwGA1UEAxMlQW50aHJvcGljIENvbnRlbnQgQ3JlZGVudGlhbHMgUm9vdCBDQTAeFw0yNjA4MDcxODQzNTZaFw0yODA4MDYxOTQzNTZaMEQxFzAVBgNVBAoTDkFudGhyb3BpYywgUEJDMSkwJwYDVQQDEyBBbnRocm9waWMgQ2xhdWRlIENvbnRlbnQgU2lnbmluZzBZMBMGByqGSM49AgEGCCqGSM49AwEHA0IABJh6CmvLUBgFFNU0vUKlOVtE6djd17L5SuwX0LemFisBM3dkd/3cyjxFA3Qo5S46fX0/ihY0VZ7mfb9KF703t5OjWDBWMA4GA1UdDwEB/wQEAwIHgDAVBgNVHSUEDjAMBgorBgEEAYPoXgIBMAwGA1UdEwEB/wQCMAAwHwYDVR0jBBgwFoAUzlHiBIFOZFsj+OPEz5o+nMHXXMIwCgYIKoZIzj0EAwMDZwAwZAIwMXMdFJ4BetLLVY7ORuE9noqbbAZOZn/aArXyTwFAZfKrPzxF2vPoJNf1+UCdg1XGAjBwX1zd9WGqYkqmL5SFqw1QySjr1zJfpJM9+1rdDwSPLMOPOjKuiXjoU/pUUeG9RwmhY3BhZFkNngAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAPZYQMr1FJyqdT6muxXqVKT5q+C5BkH/9Y2KNmSOqq1lm+fesPNihzGsYA/lWyT5IJXiuCwIBHkaz6xOJpeYCT9Y9MI=</c2pa:manifest></metadata>
<g id="graph0" class="graph" transform="scale(1 1) rotate(0) translate(28.8 677.8)">
<title>MinesweeperRobot</title>
<polygon fill="white" stroke="transparent" points="-28.8,28.8 -28.8,-677.8 1988.8,-677.8 1988.8,28.8 -28.8,28.8"/>
<text text-anchor="middle" x="980" y="-627.4" font-family="Helvetica,sans-Serif" font-size="22.00">Landmine&#45;Detection Rover — Electrical &amp; Sensor Architecture</text>
<g id="clust1" class="cluster">
<title>cluster_station</title>
<path fill="#f4f4f4" stroke="#8e8e8e" d="M756,-422.5C756,-422.5 942,-422.5 942,-422.5 948,-422.5 954,-428.5 954,-434.5 954,-434.5 954,-597 954,-597 954,-603 948,-609 942,-609 942,-609 756,-609 756,-609 750,-609 744,-603 744,-597 744,-597 744,-434.5 744,-434.5 744,-428.5 750,-422.5 756,-422.5"/>
<text text-anchor="middle" x="849" y="-593.8" font-family="Helvetica,sans-Serif" font-size="14.00">Ground Control Station</text>
</g>
<g id="clust2" class="cluster">
<title>cluster_power</title>
<path fill="#fdeaea" stroke="#c94b4b" d="M475,-294.5C475,-294.5 724,-294.5 724,-294.5 730,-294.5 736,-300.5 736,-306.5 736,-306.5 736,-485.5 736,-485.5 736,-491.5 730,-497.5 724,-497.5 724,-497.5 475,-497.5 475,-497.5 469,-497.5 463,-491.5 463,-485.5 463,-485.5 463,-306.5 463,-306.5 463,-300.5 469,-294.5 475,-294.5"/>
<text text-anchor="middle" x="599.5" y="-482.3" font-family="Helvetica,sans-Serif" font-size="14.00">Power Distribution</text>
</g>
<g id="clust3" class="cluster">
<title>cluster_compute</title>
<path fill="#eaf1ff" stroke="#3366cc" d="M782,-191C782,-191 960,-191 960,-191 966,-191 972,-197 972,-203 972,-203 972,-363 972,-363 972,-369 966,-375 960,-375 960,-375 782,-375 782,-375 776,-375 770,-369 770,-363 770,-363 770,-203 770,-203 770,-197 776,-191 782,-191"/>
<text text-anchor="middle" x="871" y="-359.8" font-family="Helvetica,sans-Serif" font-size="14.00">Compute &amp; Control</text>
</g>
<g id="clust4" class="cluster">
<title>cluster_sensors</title>
<path fill="#eafaf1" stroke="#2e8b57" d="M1047,-417C1047,-417 1940,-417 1940,-417 1946,-417 1952,-423 1952,-429 1952,-429 1952,-491 1952,-491 1952,-497 1946,-503 1940,-503 1940,-503 1047,-503 1047,-503 1041,-503 1035,-497 1035,-491 1035,-491 1035,-429 1035,-429 1035,-423 1041,-417 1047,-417"/>
<text text-anchor="middle" x="1493.5" y="-487.8" font-family="Helvetica,sans-Serif" font-size="14.00">Sensors</text>
</g>
<g id="clust5" class="cluster">
<title>cluster_actuation</title>
<path fill="#fdf6e3" stroke="#b8860b" d="M20,-8C20,-8 863,-8 863,-8 869,-8 875,-14 875,-20 875,-20 875,-148 875,-148 875,-154 869,-160 863,-160 863,-160 20,-160 20,-160 14,-160 8,-154 8,-148 8,-148 8,-20 8,-20 8,-14 14,-8 20,-8"/>
<text text-anchor="middle" x="441.5" y="-144.8" font-family="Helvetica,sans-Serif" font-size="14.00">Actuation</text>
</g>
<!-- laptop -->
<g id="node1" class="node">
<title>laptop</title>
<path fill="#dbe9ff" stroke="#3366cc" stroke-width="1.3" d="M933.5,-466.5C933.5,-466.5 764.5,-466.5 764.5,-466.5 758.5,-466.5 752.5,-460.5 752.5,-454.5 752.5,-454.5 752.5,-442.5 752.5,-442.5 752.5,-436.5 758.5,-430.5 764.5,-430.5 764.5,-430.5 933.5,-430.5 933.5,-430.5 939.5,-430.5 945.5,-436.5 945.5,-442.5 945.5,-442.5 945.5,-454.5 945.5,-454.5 945.5,-460.5 939.5,-466.5 933.5,-466.5"/>
<text text-anchor="middle" x="849" y="-451.9" font-family="Helvetica,sans-Serif" font-size="12.00">Laptop</text>
<text text-anchor="middle" x="849" y="-438.9" font-family="Helvetica,sans-Serif" font-size="12.00">(ROS Master / GUI / Video Rx)</text>
</g>
<!-- rpi -->
<g id="node10" class="node">
<title>rpi</title>
<path fill="#cfe0ff" stroke="#3366cc" stroke-width="1.3" d="M952,-344C952,-344 790,-344 790,-344 784,-344 778,-338 778,-332 778,-332 778,-309 778,-309 778,-303 784,-297 790,-297 790,-297 952,-297 952,-297 958,-297 964,-303 964,-309 964,-309 964,-332 964,-332 964,-338 958,-344 952,-344"/>
<text text-anchor="middle" x="871" y="-330.4" font-family="Helvetica,sans-Serif" font-size="12.00">Raspberry Pi</text>
<text text-anchor="middle" x="871" y="-317.4" font-family="Helvetica,sans-Serif" font-size="12.00">(ROS node host, Wi&#45;Fi link,</text>
<text text-anchor="middle" x="871" y="-304.4" font-family="Helvetica,sans-Serif" font-size="12.00">detection + EKF + mapping)</text>
</g>
<!-- laptop&#45;&gt;rpi -->
<g id="edge27" class="edge">
<title>laptop&#45;&gt;rpi</title>
<path fill="none" stroke="#3366cc" stroke-width="1.6" stroke-dasharray="5,2" d="M861.75,-420.43C861.75,-420.43 861.75,-354.13 861.75,-354.13"/>
<polygon fill="#3366cc" stroke="#3366cc" stroke-width="1.6" points="865.25,-354.13 861.75,-344.13 858.25,-354.13 865.25,-354.13"/>
<polygon fill="#3366cc" stroke="#3366cc" stroke-width="1.6" points="858.25,-420.43 861.75,-430.43 865.25,-420.43 858.25,-420.43"/>
<text text-anchor="middle" x="905.5" y="-397" font-family="Helvetica,sans-Serif" font-size="10.00">Wi&#45;Fi (ROS topics /</text>
<text text-anchor="middle" x="905.5" y="-386" font-family="Helvetica,sans-Serif" font-size="10.00">joystick teleop)</text>
</g>
<!-- joystick -->
<g id="node2" class="node">
<title>joystick</title>
<path fill="#dbe9ff" stroke="#3366cc" stroke-width="1.3" d="M902.5,-578C902.5,-578 795.5,-578 795.5,-578 789.5,-578 783.5,-572 783.5,-566 783.5,-566 783.5,-554 783.5,-554 783.5,-548 789.5,-542 795.5,-542 795.5,-542 902.5,-542 902.5,-542 908.5,-542 914.5,-548 914.5,-554 914.5,-554 914.5,-566 914.5,-566 914.5,-572 908.5,-578 902.5,-578"/>
<text text-anchor="middle" x="849" y="-556.9" font-family="Helvetica,sans-Serif" font-size="12.00">Joystick / Controller</text>
</g>
<!-- joystick&#45;&gt;laptop -->
<g id="edge1" class="edge">
<title>joystick&#45;&gt;laptop</title>
<path fill="none" stroke="#555555" stroke-width="1.4" d="M849,-541.59C849,-541.59 849,-476.57 849,-476.57"/>
<polygon fill="#555555" stroke="#555555" stroke-width="1.4" points="852.5,-476.57 849,-466.57 845.5,-476.57 852.5,-476.57"/>
<text text-anchor="middle" x="859.5" y="-514" font-family="Helvetica,sans-Serif" font-size="10.00">USB</text>
</g>
<!-- sla1 -->
<g id="node3" class="node">
<title>sla1</title>
<path fill="#fbd4d4" stroke="#c94b4b" stroke-width="1.3" d="M659,-578C659,-578 537,-578 537,-578 531,-578 525,-572 525,-566 525,-566 525,-554 525,-554 525,-548 531,-542 537,-542 537,-542 659,-542 659,-542 665,-542 671,-548 671,-554 671,-554 671,-566 671,-566 671,-572 665,-578 659,-578"/>
<text text-anchor="middle" x="598" y="-563.4" font-family="Helvetica,sans-Serif" font-size="12.00">12V Sealed Lead&#45;Acid</text>
<text text-anchor="middle" x="598" y="-550.4" font-family="Helvetica,sans-Serif" font-size="12.00">Battery #1 (35Ah)</text>
</g>
<!-- bus12v -->
<g id="node9" class="node">
<title>bus12v</title>
<polygon fill="#f7c1c1" stroke="#c94b4b" stroke-width="1.3" points="582.5,-460.5 471.5,-460.5 471.5,-436.5 582.5,-436.5 594.5,-448.5 582.5,-460.5"/>
<text text-anchor="middle" x="533" y="-451.9" font-family="Helvetica,sans-Serif" font-size="12.00">12V Bus</text>
<text text-anchor="middle" x="533" y="-438.9" font-family="Helvetica,sans-Serif" font-size="12.00">(breadboard rails)</text>
</g>
<!-- sla1&#45;&gt;bus12v -->
<g id="edge2" class="edge">
<title>sla1&#45;&gt;bus12v</title>
<path fill="none" stroke="#555555" stroke-width="1.4" d="M559.75,-541.59C559.75,-541.59 559.75,-476.57 559.75,-476.57"/>
<polygon fill="#555555" stroke="#555555" stroke-width="1.4" points="563.25,-476.57 559.75,-466.57 556.25,-476.57 563.25,-476.57"/>
</g>
<!-- sla2 -->
<g id="node4" class="node">
<title>sla2</title>
<path fill="#fbd4d4" stroke="#c94b4b" stroke-width="1.3" d="M488,-578C488,-578 366,-578 366,-578 360,-578 354,-572 354,-566 354,-566 354,-554 354,-554 354,-548 360,-542 366,-542 366,-542 488,-542 488,-542 494,-542 500,-548 500,-554 500,-554 500,-566 500,-566 500,-572 494,-578 488,-578"/>
<text text-anchor="middle" x="427" y="-563.4" font-family="Helvetica,sans-Serif" font-size="12.00">12V Sealed Lead&#45;Acid</text>
<text text-anchor="middle" x="427" y="-550.4" font-family="Helvetica,sans-Serif" font-size="12.00">Battery #2 (35Ah)</text>
</g>
<!-- sla2&#45;&gt;bus12v -->
<g id="edge3" class="edge">
<title>sla2&#45;&gt;bus12v</title>
<path fill="none" stroke="#555555" stroke-width="1.4" d="M485.75,-541.59C485.75,-541.59 485.75,-476.57 485.75,-476.57"/>
<polygon fill="#555555" stroke="#555555" stroke-width="1.4" points="489.25,-476.57 485.75,-466.57 482.25,-476.57 489.25,-476.57"/>
</g>
<!-- lipo1 -->
<g id="node5" class="node">
<title>lipo1</title>
<path fill="#fbd4d4" stroke="#c94b4b" stroke-width="1.3" d="M1092,-578C1092,-578 974,-578 974,-578 968,-578 962,-572 962,-566 962,-566 962,-554 962,-554 962,-548 968,-542 974,-542 974,-542 1092,-542 1092,-542 1098,-542 1104,-548 1104,-554 1104,-554 1104,-566 1104,-566 1104,-572 1098,-578 1092,-578"/>
<text text-anchor="middle" x="1033" y="-563.4" font-family="Helvetica,sans-Serif" font-size="12.00">3.7V LiPo 1000mAh</text>
<text text-anchor="middle" x="1033" y="-550.4" font-family="Helvetica,sans-Serif" font-size="12.00">(Raspberry Pi supply)</text>
</g>
<!-- lipo1&#45;&gt;rpi -->
<g id="edge26" class="edge">
<title>lipo1&#45;&gt;rpi</title>
<path fill="none" stroke="#555555" stroke-width="1.4" d="M961.72,-560C955.21,-560 951,-560 951,-560 951,-560 951,-354.34 951,-354.34"/>
<polygon fill="#555555" stroke="#555555" stroke-width="1.4" points="954.5,-354.34 951,-344.34 947.5,-354.34 954.5,-354.34"/>
<text text-anchor="middle" x="1007" y="-446" font-family="Helvetica,sans-Serif" font-size="10.00">3.7V</text>
</g>
<!-- lipo2 -->
<g id="node6" class="node">
<title>lipo2</title>
<path fill="#fbd4d4" stroke="#c94b4b" stroke-width="1.3" d="M167.5,-578C167.5,-578 38.5,-578 38.5,-578 32.5,-578 26.5,-572 26.5,-566 26.5,-566 26.5,-554 26.5,-554 26.5,-548 32.5,-542 38.5,-542 38.5,-542 167.5,-542 167.5,-542 173.5,-542 179.5,-548 179.5,-554 179.5,-554 179.5,-566 179.5,-566 179.5,-572 173.5,-578 167.5,-578"/>
<text text-anchor="middle" x="103" y="-563.4" font-family="Helvetica,sans-Serif" font-size="12.00">3.7V LiPo 2000mAh</text>
<text text-anchor="middle" x="103" y="-550.4" font-family="Helvetica,sans-Serif" font-size="12.00">(Gripper motor supply)</text>
</g>
<!-- mdd3 -->
<g id="node19" class="node">
<title>mdd3</title>
<path fill="#faecc4" stroke="#b8860b" stroke-width="1.3" d="M177.5,-129C177.5,-129 28.5,-129 28.5,-129 22.5,-129 16.5,-123 16.5,-117 16.5,-117 16.5,-105 16.5,-105 16.5,-99 22.5,-93 28.5,-93 28.5,-93 177.5,-93 177.5,-93 183.5,-93 189.5,-99 189.5,-105 189.5,-105 189.5,-117 189.5,-117 189.5,-123 183.5,-129 177.5,-129"/>
<text text-anchor="middle" x="103" y="-114.4" font-family="Helvetica,sans-Serif" font-size="12.00">MDD10A #3</text>
<text text-anchor="middle" x="103" y="-101.4" font-family="Helvetica,sans-Serif" font-size="12.00">(Dual&#45;Channel 10A Driver)</text>
</g>
<!-- lipo2&#45;&gt;mdd3 -->
<g id="edge23" class="edge">
<title>lipo2&#45;&gt;mdd3</title>
<path fill="none" stroke="#555555" stroke-width="1.4" d="M103,-541.84C103,-541.84 103,-139.38 103,-139.38"/>
<polygon fill="#555555" stroke="#555555" stroke-width="1.4" points="106.5,-139.38 103,-129.38 99.5,-139.38 106.5,-139.38"/>
<text text-anchor="middle" x="118.5" y="-318" font-family="Helvetica,sans-Serif" font-size="10.00">power</text>
</g>
<!-- buck -->
<g id="node7" class="node">
<title>buck</title>
<path fill="#fbd4d4" stroke="#c94b4b" stroke-width="1.3" d="M716,-338.5C716,-338.5 604,-338.5 604,-338.5 598,-338.5 592,-332.5 592,-326.5 592,-326.5 592,-314.5 592,-314.5 592,-308.5 598,-302.5 604,-302.5 604,-302.5 716,-302.5 716,-302.5 722,-302.5 728,-308.5 728,-314.5 728,-314.5 728,-326.5 728,-326.5 728,-332.5 722,-338.5 716,-338.5"/>
<text text-anchor="middle" x="660" y="-323.9" font-family="Helvetica,sans-Serif" font-size="12.00">Buck Converter</text>
<text text-anchor="middle" x="660" y="-310.9" font-family="Helvetica,sans-Serif" font-size="12.00">(12V → 5V logic rail)</text>
</g>
<!-- buck&#45;&gt;rpi -->
<g id="edge24" class="edge">
<title>buck&#45;&gt;rpi</title>
<path fill="none" stroke="#555555" stroke-width="1.4" d="M728.21,-320C728.21,-320 767.68,-320 767.68,-320"/>
<polygon fill="#555555" stroke="#555555" stroke-width="1.4" points="767.68,-323.5 777.68,-320 767.68,-316.5 767.68,-323.5"/>
</g>
<!-- arduino -->
<g id="node11" class="node">
<title>arduino</title>
<path fill="#cfe0ff" stroke="#3366cc" stroke-width="1.3" d="M949.5,-246C949.5,-246 790.5,-246 790.5,-246 784.5,-246 778.5,-240 778.5,-234 778.5,-234 778.5,-211 778.5,-211 778.5,-205 784.5,-199 790.5,-199 790.5,-199 949.5,-199 949.5,-199 955.5,-199 961.5,-205 961.5,-211 961.5,-211 961.5,-234 961.5,-234 961.5,-240 955.5,-246 949.5,-246"/>
<text text-anchor="middle" x="870" y="-232.4" font-family="Helvetica,sans-Serif" font-size="12.00">Arduino Uno</text>
<text text-anchor="middle" x="870" y="-219.4" font-family="Helvetica,sans-Serif" font-size="12.00">(low&#45;level motor PWM,</text>
<text text-anchor="middle" x="870" y="-206.4" font-family="Helvetica,sans-Serif" font-size="12.00">encoder counting, coil ADC)</text>
</g>
<!-- buck&#45;&gt;arduino -->
<g id="edge25" class="edge">
<title>buck&#45;&gt;arduino</title>
<path fill="none" stroke="#555555" stroke-width="1.4" d="M710.75,-302.16C710.75,-277.31 710.75,-236 710.75,-236 710.75,-236 768.29,-236 768.29,-236"/>
<polygon fill="#555555" stroke="#555555" stroke-width="1.4" points="768.29,-239.5 778.29,-236 768.29,-232.5 768.29,-239.5"/>
<text text-anchor="middle" x="749.5" y="-269" font-family="Helvetica,sans-Serif" font-size="10.00">5V</text>
</g>
<!-- relay -->
<g id="node8" class="node">
<title>relay</title>
<path fill="#fbd4d4" stroke="#c94b4b" stroke-width="1.3" d="M555,-338.5C555,-338.5 483,-338.5 483,-338.5 477,-338.5 471,-332.5 471,-326.5 471,-326.5 471,-314.5 471,-314.5 471,-308.5 477,-302.5 483,-302.5 483,-302.5 555,-302.5 555,-302.5 561,-302.5 567,-308.5 567,-314.5 567,-314.5 567,-326.5 567,-326.5 567,-332.5 561,-338.5 555,-338.5"/>
<text text-anchor="middle" x="519" y="-323.9" font-family="Helvetica,sans-Serif" font-size="12.00">Relay Module</text>
<text text-anchor="middle" x="519" y="-310.9" font-family="Helvetica,sans-Serif" font-size="12.00">(2&#45;channel)</text>
</g>
<!-- electromagnet -->
<g id="node22" class="node">
<title>electromagnet</title>
<path fill="#fff2cc" stroke="#b8860b" stroke-width="1.3" d="M458,-129C458,-129 378,-129 378,-129 372,-129 366,-123 366,-117 366,-117 366,-105 366,-105 366,-99 372,-93 378,-93 378,-93 458,-93 458,-93 464,-93 470,-99 470,-105 470,-105 470,-117 470,-117 470,-123 464,-129 458,-129"/>
<text text-anchor="middle" x="418" y="-114.4" font-family="Helvetica,sans-Serif" font-size="12.00">Electromagnet</text>
<text text-anchor="middle" x="418" y="-101.4" font-family="Helvetica,sans-Serif" font-size="12.00">(end&#45;effector)</text>
</g>
<!-- relay&#45;&gt;electromagnet -->
<g id="edge18" class="edge">
<title>relay&#45;&gt;electromagnet</title>
<path fill="none" stroke="#555555" stroke-width="1.4" d="M483.25,-302.27C483.25,-251.3 483.25,-111 483.25,-111 483.25,-111 480.05,-111 480.05,-111"/>
<polygon fill="#555555" stroke="#555555" stroke-width="1.4" points="480.05,-107.5 470.05,-111 480.05,-114.5 480.05,-107.5"/>
<text text-anchor="middle" x="529" y="-220" font-family="Helvetica,sans-Serif" font-size="10.00">switched 12V</text>
</g>
<!-- siren -->
<g id="node23" class="node">
<title>siren</title>
<path fill="#fff2cc" stroke="#b8860b" stroke-width="1.3" d="M329,-129C329,-129 227,-129 227,-129 221,-129 215,-123 215,-117 215,-117 215,-105 215,-105 215,-99 221,-93 227,-93 227,-93 329,-93 329,-93 335,-93 341,-99 341,-105 341,-105 341,-117 341,-117 341,-123 335,-129 329,-129"/>
<text text-anchor="middle" x="278" y="-114.4" font-family="Helvetica,sans-Serif" font-size="12.00">Siren &amp; Buzzer</text>
<text text-anchor="middle" x="278" y="-101.4" font-family="Helvetica,sans-Serif" font-size="12.00">(mine&#45;found alert)</text>
</g>
<!-- relay&#45;&gt;siren -->
<g id="edge19" class="edge">
<title>relay&#45;&gt;siren</title>
<path fill="none" stroke="#555555" stroke-width="1.4" d="M470.81,-320C400.48,-320 278,-320 278,-320 278,-320 278,-139.18 278,-139.18"/>
<polygon fill="#555555" stroke="#555555" stroke-width="1.4" points="281.5,-139.18 278,-129.18 274.5,-139.18 281.5,-139.18"/>
<text text-anchor="middle" x="311" y="-220" font-family="Helvetica,sans-Serif" font-size="10.00">switched 12V</text>
</g>
<!-- bus12v&#45;&gt;buck -->
<g id="edge4" class="edge">
<title>bus12v&#45;&gt;buck</title>
<path fill="none" stroke="#555555" stroke-width="1.4" d="M583.67,-430.36C583.67,-394.94 583.67,-320 583.67,-320 583.67,-320 584.48,-320 584.48,-320"/>
<polygon fill="#555555" stroke="#555555" stroke-width="1.4" points="581.84,-323.5 591.84,-320 581.84,-316.5 581.84,-323.5"/>
<text text-anchor="middle" x="630.5" y="-391.5" font-family="Helvetica,sans-Serif" font-size="10.00">12V</text>
</g>
<!-- bus12v&#45;&gt;relay -->
<g id="edge5" class="edge">
<title>bus12v&#45;&gt;relay</title>
<path fill="none" stroke="#555555" stroke-width="1.4" d="M519.25,-430.43C519.25,-430.43 519.25,-348.89 519.25,-348.89"/>
<polygon fill="#555555" stroke="#555555" stroke-width="1.4" points="522.75,-348.89 519.25,-338.89 515.75,-348.89 522.75,-348.89"/>
<text text-anchor="middle" x="535.5" y="-391.5" font-family="Helvetica,sans-Serif" font-size="10.00">12V</text>
</g>
<!-- mdd1 -->
<g id="node17" class="node">
<title>mdd1</title>
<path fill="#faecc4" stroke="#b8860b" stroke-width="1.3" d="M854.5,-129C854.5,-129 705.5,-129 705.5,-129 699.5,-129 693.5,-123 693.5,-117 693.5,-117 693.5,-105 693.5,-105 693.5,-99 699.5,-93 705.5,-93 705.5,-93 854.5,-93 854.5,-93 860.5,-93 866.5,-99 866.5,-105 866.5,-105 866.5,-117 866.5,-117 866.5,-123 860.5,-129 854.5,-129"/>
<text text-anchor="middle" x="780" y="-114.4" font-family="Helvetica,sans-Serif" font-size="12.00">MDD10A #1</text>
<text text-anchor="middle" x="780" y="-101.4" font-family="Helvetica,sans-Serif" font-size="12.00">(Dual&#45;Channel 10A Driver)</text>
</g>
<!-- bus12v&#45;&gt;mdd1 -->
<g id="edge21" class="edge">
<title>bus12v&#45;&gt;mdd1</title>
<path fill="none" stroke="#555555" stroke-width="1.4" d="M594.62,-448C655.71,-448 740.25,-448 740.25,-448 740.25,-448 740.25,-139.12 740.25,-139.12"/>
<polygon fill="#555555" stroke="#555555" stroke-width="1.4" points="743.75,-139.12 740.25,-129.12 736.75,-139.12 743.75,-139.12"/>
<text text-anchor="middle" x="442.5" y="-269" font-family="Helvetica,sans-Serif" font-size="10.00">12V</text>
</g>
<!-- mdd2 -->
<g id="node18" class="node">
<title>mdd2</title>
<path fill="#faecc4" stroke="#b8860b" stroke-width="1.3" d="M656.5,-129C656.5,-129 507.5,-129 507.5,-129 501.5,-129 495.5,-123 495.5,-117 495.5,-117 495.5,-105 495.5,-105 495.5,-99 501.5,-93 507.5,-93 507.5,-93 656.5,-93 656.5,-93 662.5,-93 668.5,-99 668.5,-105 668.5,-105 668.5,-117 668.5,-117 668.5,-123 662.5,-129 656.5,-129"/>
<text text-anchor="middle" x="582" y="-114.4" font-family="Helvetica,sans-Serif" font-size="12.00">MDD10A #2</text>
<text text-anchor="middle" x="582" y="-101.4" font-family="Helvetica,sans-Serif" font-size="12.00">(Dual&#45;Channel 10A Driver)</text>
</g>
<!-- bus12v&#45;&gt;mdd2 -->
<g id="edge22" class="edge">
<title>bus12v&#45;&gt;mdd2</title>
<path fill="none" stroke="#555555" stroke-width="1.4" d="M575.33,-430.35C575.33,-430.35 575.33,-139.24 575.33,-139.24"/>
<polygon fill="#555555" stroke="#555555" stroke-width="1.4" points="578.83,-139.24 575.33,-129.24 571.83,-139.24 578.83,-139.24"/>
<text text-anchor="middle" x="391.5" y="-269" font-family="Helvetica,sans-Serif" font-size="10.00">12V</text>
</g>
<!-- rpi&#45;&gt;arduino -->
<g id="edge6" class="edge">
<title>rpi&#45;&gt;arduino</title>
<path fill="none" stroke="#555555" stroke-width="1.4" d="M870,-296.78C870,-296.78 870,-256 870,-256"/>
<polygon fill="#555555" stroke="#555555" stroke-width="1.4" points="873.5,-256 870,-246 866.5,-256 873.5,-256"/>
<text text-anchor="middle" x="895.5" y="-269" font-family="Helvetica,sans-Serif" font-size="10.00">USB serial</text>
</g>
<!-- arduino&#45;&gt;relay -->
<g id="edge20" class="edge">
<title>arduino&#45;&gt;relay</title>
<path fill="none" stroke="#555555" stroke-width="1.4" d="M778.33,-227C678.01,-227 531.25,-227 531.25,-227 531.25,-227 531.25,-292.47 531.25,-292.47"/>
<polygon fill="#555555" stroke="#555555" stroke-width="1.4" points="527.75,-292.47 531.25,-302.47 534.75,-292.47 527.75,-292.47"/>
<text text-anchor="middle" x="667.5" y="-269" font-family="Helvetica,sans-Serif" font-size="10.00">control signal</text>
</g>
<!-- arduino&#45;&gt;mdd1 -->
<g id="edge12" class="edge">
<title>arduino&#45;&gt;mdd1</title>
<path fill="none" stroke="#555555" stroke-width="1.4" d="M822.5,-198.85C822.5,-198.85 822.5,-139.11 822.5,-139.11"/>
<polygon fill="#555555" stroke="#555555" stroke-width="1.4" points="826,-139.11 822.5,-129.11 819,-139.11 826,-139.11"/>
<text text-anchor="middle" x="848" y="-171" font-family="Helvetica,sans-Serif" font-size="10.00">PWM/DIR</text>
</g>
<!-- arduino&#45;&gt;mdd2 -->
<g id="edge13" class="edge">
<title>arduino&#45;&gt;mdd2</title>
<path fill="none" stroke="#555555" stroke-width="1.4" d="M778.33,-208C710.65,-208 630.25,-208 630.25,-208 630.25,-208 630.25,-139.24 630.25,-139.24"/>
<polygon fill="#555555" stroke="#555555" stroke-width="1.4" points="633.75,-139.24 630.25,-129.24 626.75,-139.24 633.75,-139.24"/>
<text text-anchor="middle" x="711" y="-171" font-family="Helvetica,sans-Serif" font-size="10.00">PWM/DIR</text>
</g>
<!-- arduino&#45;&gt;mdd3 -->
<g id="edge14" class="edge">
<title>arduino&#45;&gt;mdd3</title>
<path fill="none" stroke="#555555" stroke-width="1.4" d="M778.26,-217C591.47,-217 184.5,-217 184.5,-217 184.5,-217 184.5,-139.46 184.5,-139.46"/>
<polygon fill="#555555" stroke="#555555" stroke-width="1.4" points="188,-139.46 184.5,-129.46 181,-139.46 188,-139.46"/>
<text text-anchor="middle" x="448" y="-171" font-family="Helvetica,sans-Serif" font-size="10.00">PWM/DIR</text>
</g>
<!-- camera -->
<g id="node12" class="node">
<title>camera</title>
<path fill="#c9f2da" stroke="#2e8b57" stroke-width="1.3" d="M1205,-472C1205,-472 1055,-472 1055,-472 1049,-472 1043,-466 1043,-460 1043,-460 1043,-437 1043,-437 1043,-431 1049,-425 1055,-425 1055,-425 1205,-425 1205,-425 1211,-425 1217,-431 1217,-437 1217,-437 1217,-460 1217,-460 1217,-466 1211,-472 1205,-472"/>
<text text-anchor="middle" x="1130" y="-458.4" font-family="Helvetica,sans-Serif" font-size="12.00">Pi Camera</text>
<text text-anchor="middle" x="1130" y="-445.4" font-family="Helvetica,sans-Serif" font-size="12.00">(YOLOv8 mine detection,</text>
<text text-anchor="middle" x="1130" y="-432.4" font-family="Helvetica,sans-Serif" font-size="12.00">GStreamer RTSP ~100ms)</text>
</g>
<!-- camera&#45;&gt;rpi -->
<g id="edge7" class="edge">
<title>camera&#45;&gt;rpi</title>
<path fill="none" stroke="#555555" stroke-width="1.4" d="M1033,-448C1033,-448 956.5,-448 956.5,-448 956.5,-448 956.5,-381.06 956.5,-344.18"/>
<polygon fill="#555555" stroke="#555555" stroke-width="1.4" points="1033,-451.5 1043,-448 1033,-444.5 1033,-451.5"/>
<text text-anchor="middle" x="1107" y="-391.5" font-family="Helvetica,sans-Serif" font-size="10.00">CSI / USB</text>
</g>
<!-- coil -->
<g id="node13" class="node">
<title>coil</title>
<path fill="#c9f2da" stroke="#2e8b57" stroke-width="1.3" d="M1547.5,-466.5C1547.5,-466.5 1434.5,-466.5 1434.5,-466.5 1428.5,-466.5 1422.5,-460.5 1422.5,-454.5 1422.5,-454.5 1422.5,-442.5 1422.5,-442.5 1422.5,-436.5 1428.5,-430.5 1434.5,-430.5 1434.5,-430.5 1547.5,-430.5 1547.5,-430.5 1553.5,-430.5 1559.5,-436.5 1559.5,-442.5 1559.5,-442.5 1559.5,-454.5 1559.5,-454.5 1559.5,-460.5 1553.5,-466.5 1547.5,-466.5"/>
<text text-anchor="middle" x="1491" y="-451.9" font-family="Helvetica,sans-Serif" font-size="12.00">Metal&#45;Detecting Coil</text>
<text text-anchor="middle" x="1491" y="-438.9" font-family="Helvetica,sans-Serif" font-size="12.00">(mine sensor)</text>
</g>
<!-- coil&#45;&gt;arduino -->
<g id="edge8" class="edge">
<title>coil&#45;&gt;arduino</title>
<path fill="none" stroke="#555555" stroke-width="1.4" d="M1491,-420.37C1491,-420.37 1491,-227 1491,-227 1491,-227 1134.37,-227 961.63,-227"/>
<polygon fill="#555555" stroke="#555555" stroke-width="1.4" points="1487.5,-420.37 1491,-430.37 1494.5,-420.37 1487.5,-420.37"/>
<text text-anchor="middle" x="1438" y="-318" font-family="Helvetica,sans-Serif" font-size="10.00">analog</text>
</g>
<!-- imu -->
<g id="node14" class="node">
<title>imu</title>
<path fill="#c9f2da" stroke="#2e8b57" stroke-width="1.3" d="M1723,-466.5C1723,-466.5 1597,-466.5 1597,-466.5 1591,-466.5 1585,-460.5 1585,-454.5 1585,-454.5 1585,-442.5 1585,-442.5 1585,-436.5 1591,-430.5 1597,-430.5 1597,-430.5 1723,-430.5 1723,-430.5 1729,-430.5 1735,-436.5 1735,-442.5 1735,-442.5 1735,-454.5 1735,-454.5 1735,-460.5 1729,-466.5 1723,-466.5"/>
<text text-anchor="middle" x="1660" y="-451.9" font-family="Helvetica,sans-Serif" font-size="12.00">IMU — MPU6050/9250</text>
<text text-anchor="middle" x="1660" y="-438.9" font-family="Helvetica,sans-Serif" font-size="12.00">(GY&#45;91, I2C)</text>
</g>
<!-- imu&#45;&gt;arduino -->
<g id="edge9" class="edge">
<title>imu&#45;&gt;arduino</title>
<path fill="none" stroke="#555555" stroke-width="1.4" d="M1660,-420.33C1660,-420.33 1660,-217 1660,-217 1660,-217 1169.96,-217 961.78,-217"/>
<polygon fill="#555555" stroke="#555555" stroke-width="1.4" points="1656.5,-420.33 1660,-430.33 1663.5,-420.33 1656.5,-420.33"/>
<text text-anchor="middle" x="1631" y="-318" font-family="Helvetica,sans-Serif" font-size="10.00">I2C</text>
</g>
<!-- encoders -->
<g id="node15" class="node">
<title>encoders</title>
<path fill="#c9f2da" stroke="#2e8b57" stroke-width="1.3" d="M1931.5,-466.5C1931.5,-466.5 1772.5,-466.5 1772.5,-466.5 1766.5,-466.5 1760.5,-460.5 1760.5,-454.5 1760.5,-454.5 1760.5,-442.5 1760.5,-442.5 1760.5,-436.5 1766.5,-430.5 1772.5,-430.5 1772.5,-430.5 1931.5,-430.5 1931.5,-430.5 1937.5,-430.5 1943.5,-436.5 1943.5,-442.5 1943.5,-442.5 1943.5,-454.5 1943.5,-454.5 1943.5,-460.5 1937.5,-466.5 1931.5,-466.5"/>
<text text-anchor="middle" x="1852" y="-451.9" font-family="Helvetica,sans-Serif" font-size="12.00">Wheel Encoders</text>
<text text-anchor="middle" x="1852" y="-438.9" font-family="Helvetica,sans-Serif" font-size="12.00">(independent tire encoders)</text>
</g>
<!-- encoders&#45;&gt;arduino -->
<g id="edge10" class="edge">
<title>encoders&#45;&gt;arduino</title>
<path fill="none" stroke="#555555" stroke-width="1.4" d="M1852,-420.42C1852,-420.42 1852,-208 1852,-208 1852,-208 1205.28,-208 961.5,-208"/>
<polygon fill="#555555" stroke="#555555" stroke-width="1.4" points="1848.5,-420.42 1852,-430.42 1855.5,-420.42 1848.5,-420.42"/>
<text text-anchor="middle" x="1844" y="-318" font-family="Helvetica,sans-Serif" font-size="10.00">digital pulses</text>
</g>
<!-- pot1 -->
<g id="node16" class="node">
<title>pot1</title>
<path fill="#c9f2da" stroke="#2e8b57" stroke-width="1.3" d="M1385.5,-466.5C1385.5,-466.5 1254.5,-466.5 1254.5,-466.5 1248.5,-466.5 1242.5,-460.5 1242.5,-454.5 1242.5,-454.5 1242.5,-442.5 1242.5,-442.5 1242.5,-436.5 1248.5,-430.5 1254.5,-430.5 1254.5,-430.5 1385.5,-430.5 1385.5,-430.5 1391.5,-430.5 1397.5,-436.5 1397.5,-442.5 1397.5,-442.5 1397.5,-454.5 1397.5,-454.5 1397.5,-460.5 1391.5,-466.5 1385.5,-466.5"/>
<text text-anchor="middle" x="1320" y="-451.9" font-family="Helvetica,sans-Serif" font-size="12.00">Potentiometer /</text>
<text text-anchor="middle" x="1320" y="-438.9" font-family="Helvetica,sans-Serif" font-size="12.00">Gripper position sensor</text>
</g>
<!-- pot1&#45;&gt;arduino -->
<g id="edge11" class="edge">
<title>pot1&#45;&gt;arduino</title>
<path fill="none" stroke="#555555" stroke-width="1.4" d="M1320,-420.38C1320,-420.38 1320,-236 1320,-236 1320,-236 1093.18,-236 961.61,-236"/>
<polygon fill="#555555" stroke="#555555" stroke-width="1.4" points="1316.5,-420.38 1320,-430.38 1323.5,-420.38 1316.5,-420.38"/>
<text text-anchor="middle" x="1202" y="-318" font-family="Helvetica,sans-Serif" font-size="10.00">analog</text>
</g>
<!-- drive_motors -->
<g id="node20" class="node">
<title>drive_motors</title>
<path fill="#fff2cc" stroke="#b8860b" stroke-width="1.3" d="M736.5,-52C736.5,-52 625.5,-52 625.5,-52 619.5,-52 613.5,-46 613.5,-40 613.5,-40 613.5,-28 613.5,-28 613.5,-22 619.5,-16 625.5,-16 625.5,-16 736.5,-16 736.5,-16 742.5,-16 748.5,-22 748.5,-28 748.5,-28 748.5,-40 748.5,-40 748.5,-46 742.5,-52 736.5,-52"/>
<text text-anchor="middle" x="681" y="-37.4" font-family="Helvetica,sans-Serif" font-size="12.00">4× DC Drive Motors</text>
<text text-anchor="middle" x="681" y="-24.4" font-family="Helvetica,sans-Serif" font-size="12.00">(110 RPM, wheels)</text>
</g>
<!-- mdd1&#45;&gt;drive_motors -->
<g id="edge15" class="edge">
<title>mdd1&#45;&gt;drive_motors</title>
<path fill="none" stroke="#555555" stroke-width="1.4" d="M721,-92.75C721,-92.75 721,-62.12 721,-62.12"/>
<polygon fill="#555555" stroke="#555555" stroke-width="1.4" points="724.5,-62.12 721,-52.12 717.5,-62.12 724.5,-62.12"/>
</g>
<!-- mdd2&#45;&gt;drive_motors -->
<g id="edge16" class="edge">
<title>mdd2&#45;&gt;drive_motors</title>
<path fill="none" stroke="#555555" stroke-width="1.4" d="M641,-92.75C641,-92.75 641,-62.12 641,-62.12"/>
<polygon fill="#555555" stroke="#555555" stroke-width="1.4" points="644.5,-62.12 641,-52.12 637.5,-62.12 644.5,-62.12"/>
</g>
<!-- gripper_motor -->
<g id="node21" class="node">
<title>gripper_motor</title>
<path fill="#fff2cc" stroke="#b8860b" stroke-width="1.3" d="M155,-52C155,-52 51,-52 51,-52 45,-52 39,-46 39,-40 39,-40 39,-28 39,-28 39,-22 45,-16 51,-16 51,-16 155,-16 155,-16 161,-16 167,-22 167,-28 167,-28 167,-40 167,-40 167,-46 161,-52 155,-52"/>
<text text-anchor="middle" x="103" y="-37.4" font-family="Helvetica,sans-Serif" font-size="12.00">Gripper&#45;Arm Motor</text>
<text text-anchor="middle" x="103" y="-24.4" font-family="Helvetica,sans-Serif" font-size="12.00">(high&#45;torque)</text>
</g>
<!-- mdd3&#45;&gt;gripper_motor -->
<g id="edge17" class="edge">
<title>mdd3&#45;&gt;gripper_motor</title>
<path fill="none" stroke="#555555" stroke-width="1.4" d="M103,-92.75C103,-92.75 103,-62.12 103,-62.12"/>
<polygon fill="#555555" stroke="#555555" stroke-width="1.4" points="106.5,-62.12 103,-52.12 99.5,-62.12 106.5,-62.12"/>
</g>
</g>
</svg>
