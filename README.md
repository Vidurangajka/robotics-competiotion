# 🤖 Robotic Design Project – EN2533

![Project Banner](https://github.com/user-attachments/assets/4f7ffe4b-5368-4e45-8c46-ff8df11e428c)

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
- [👨‍💻 Task Allocation](#-task-allocation)
- [📸 Gallery](#-gallery)

---

## 📦 Project Overview

This project aims to design and build a fully autonomous robot capable of navigating complex environments and performing multi-step tasks. It was developed for the **EN2533 Robot Design and Competition** course. The robot uses modular sensor integration, precision actuation, and robust control algorithms.

> The robot sequentially tackles tasks like barcode reading, maze navigation, object sorting, and even coin dropping, utilizing sophisticated sensor arrays and a SolidWorks-designed robotic arm.

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
- **Compact build** for stability and easy modular assembly
- **Mounting rails** for sensor arrays

<img src="https://github.com/user-attachments/assets/486dd6bb-67a7-4aad-bec2-bbbca92b563e" width="600"/>

---

## 🧠 Algorithms and Tasks

### 🧮 Task 01 – Bar Counting and Line Navigation

The robot reads barcodes by detecting white duration using IR sensors. Barcode ends after 3 consecutive zeros.

![Task 1](https://github.com/user-attachments/assets/3bdcc0bd-dae0-4d4a-b734-00ec39e764e4)

---

### 📦 Task 02 – Virtual Box Movement & Maze Navigation

- Detects box
- Chooses open gate (red/blue)
- Follows optimal path using encoder + PD correction

![Task 2](https://github.com/user-attachments/assets/c0cdbbd9-ef66-414c-9092-fa4d3ee1d0ee)

---

### 🎨 Task 03 – Color Line Following

TCS34725 checks hue, while IR array ensures alignment.

![Task 3](https://github.com/user-attachments/assets/5b40057c-b353-4f27-b726-fb0e1cd15086)

---

### ⚪ Task 04 – Dotted Line Following

Detects white dashes and navigates accordingly. Stops when all IR sensors read white.

![Task 4](https://github.com/user-attachments/assets/15201817-8344-48cb-a39f-e4b60a210b69)

---

### 🚪 Task 05 – Portal Navigation

ToF monitors gate distance. Robot advances when safe. Ultrasonic confirms no obstacles.

![Task 5](https://github.com/user-attachments/assets/012867a9-86f8-46b3-94ed-5df48ed2c334)

---

### 📦 Task 06 – Box Height Sorting

Grabs boxes of 5/10/15cm height and arranges based on detected color.

![Task 6](https://github.com/user-attachments/assets/4c62954c-45f7-4a66-ad31-e93b77477629)

---

### 🕳️ Task 07 – Chamber Insertion

Box is pushed inside a detected chamber using the gripper after ultrasonic alignment.

![Task 7](https://github.com/user-attachments/assets/9ea790fe-4283-404c-8dfe-f736f8cec3f3)

---

### 🪙 Task 08 – Coin Drop

Detects black cross on rough terrain and releases a coin using sliding mech.

![Task 8](https://github.com/user-attachments/assets/e3efcc74-0031-4a54-9ba4-f4dcbe90c70b)

---

## 👨‍💻 Task Allocation

| Member | ID | Contributions |
|--------|----|---------------|
| **Eshan S.G.S** | 220148G | Task 2, 3, 4, 8; Line following; Maze algorithm; Testing |
| **Manawadu M.D** | 220381M | Task 6, 7; Robot arm design; Box sorting logic |
| **Wickramasinghe S.D** | 220701X | Task 1; Power circuits; Robot assembly; Coin mech |
| **Gunasekara V.G.V** | 220193M | SolidWorks; Task 6, 7 coding; Arm assembly |
| **Ravishan B.B.N** | 220533H | Task 5; Circuit design; Robot testing |

---

## 📸 Gallery

| Design Snapshots | Arm Close-up |
|------------------|--------------|
| ![chassis](https://github.com/user-attachments/assets/3f2d229a-93eb-4e14-bb0d-99c6b06aea9a) | ![arm](https://github.com/user-attachments/assets/ad90bc2f-506b-4973-aa41-bd4042350150) |

---

## 🔗 Resources

- 📄 [Task 2024 Specification](https://online.uom.lk/mod/resource/view.php?id=432368)
- 🛠️ SolidWorks Files: _Coming soon_
- 💻 Firmware: _Coming soon_
- 📹 Video Demo: _Coming soon_

---

## ⭐ Credits

University of Moratuwa – Department of Mechanical Engineering  
**Course**: EN2533 Robot Design and Competition  
**Mentor**: _[Add your supervisor’s name if applicable]_

---

