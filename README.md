# 🤖 Robotic Design Project – EN2533

<p align="center">
  <b>Modular, autonomous robot designed to handle multiple real-world challenges as part of the EN2533 Robot Design and Competition module.</b>
</p>

---

## 📌 Table of Contents

- [📦 Project Overview](#-project-overview)
- [🚀 Features](#-features)
- [⚙️ Hardware Design](#️-hardware-design)
- [🛠️ Mechanical Structure](#️-mechanical-structure)
- [🧠 Algorithms and Tasks](#-algorithms-and-tasks)
- [🔗 Resources](#-resources)

---

## 📦 Project Overview

This project aims to design and build a fully autonomous robot capable of navigating complex environments and performing multi-step tasks. It was developed for the **EN2533 Robot Design and Competition** course at the University of Moratuwa.

The robot performs tasks like barcode reading, maze solving, color-based line following, object manipulation, and precise coin dropping using an integrated sensor-actuator system and custom mechanical design.

---

## 🚀 Features

- 🔍 **Bar Counting** using IR sensors
- 🎨 **Color Line Following** with TCS34725
- 📏 **Box Height Detection & Sorting**
- 🧩 **Maze Navigation** with hardcoded optimal paths
- 🧠 **Portal Detection** using ToF & ultrasonic fusion
- 🎯 **Coin Drop Mechanism**
- 🚪 **Chamber Box Insertion**
- ✳️ **Dotted Line Tracking**
- 🧲 **Gripper-Based Object Handling**

---

## ⚙️ Hardware Design

### 🔌 Sensors
| Sensor | Purpose |
|--------|---------|
| **Ultrasonic Sensors** | Wall/chamber detection |
| **Sharp IR Sensor** | Height detection |
| **ToF Sensor** | Portal distance measurement |
| **TCS34725** | Color line following |
| **TCRT5000 IR Array** | Line and dashed line tracking |

### 🔩 Actuators
- **2x MG90S Servo Motors** – Gripper and slider
- **2x JGA25-370 DC Motors** – Drive wheels

### 💻 Microcontroller
- **Arduino Mega 2560** – Handles all sensor and actuator logic

---

## 🛠️ Mechanical Structure

- **Custom SolidWorks Design**
  - Two-layer acrylic chassis
  - Precision rack-and-pinion robotic arm
- **Compact and modular build** for optimized space and stability

---

## 🧠 Algorithms and Tasks

### 🧮 Task 01 – Bar Counting and Line Navigation
The robot reads barcodes by detecting white duration using IR sensors. Barcode ends after 3 consecutive zeros.

---

### 📦 Task 02 – Virtual Box Movement & Maze Navigation
- Detects virtual box
- Chooses open gate (red/blue)
- Follows predefined path using encoder + PD control

---

### 🎨 Task 03 – Color Line Following
TCS34725 checks hue, while IR array ensures line tracking.

---

### ⚪ Task 04 – Dotted Line Following
Detects white dashes, adjusts trajectory, and stops when all sensors detect white.

---

### 🚪 Task 05 – Portal Navigation
ToF monitors gate distance. Robot advances when safe. Ultrasonic confirms obstacle-free path.

---

### 📦 Task 06 – Box Height Sorting
Detects heights (5cm, 10cm, 15cm) using IR sensor. Boxes sorted based on height and prior color detection.

---

### 🕳️ Task 07 – Chamber Insertion
Box is pushed into the chamber using a closed gripper. Chamber detected via ultrasonic sensing.

---

### 🪙 Task 08 – Coin Drop
On detecting a black cross in uneven terrain, robot releases a coin using a sliding mechanism.

---

## 🔗 Resources

- 📄 [Task 2024 Specification](https://online.uom.lk/mod/resource/view.php?id=432368)
- 🛠️ SolidWorks Files: _Coming soon_
- 💻 Firmware Code: _Coming soon_
- 📹 Video Demonstration: _Coming soon_

---

## 🏫 Acknowledgments

University of Moratuwa – Department of Mechanical Engineering  
**Course**: EN2533 – Robot Design and Competition  
