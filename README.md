# Smart Endoscopic Autonomous System: One-Port Continuum Surgical Robot

[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange.svg)](https://gazebosim.org/)
[![Python](https://img.shields.io/badge/Python-3.12-blue.svg)](https://www.python.org/)

## Overview
This repository contains the software simulation, mathematical kinematics solver, and hardware control architecture for a one-port tendon-driven continuum surgical robot. Developed as a graduation project by Team Four from the Systems and Biomedical Engineering department at Cairo University, this cyber-physical system bridges a physical prototype (designed with a specialized anti-twist backbone) with a highly accurate digital twin.

**Funding & Support:** This project was developed with support from ITIDA's ITAC funding program (2025–2026).

## Team Members
* **Ahmed Etman**
* **Ziad Mohamed**
* **Heidi Hussain**
* **Adham Khaled**

**Project Advisor:** Assistant Professor Mohamed Islam

# System Overview
<p align="center">
  <img src="Images/gazebo_simulation_gui.png" width="45%" title="System Overview" />
  <img src="Images/IMG_20260627_190525.jpeg" width="45%" title="Hardware Prototype" />
</p> 

---

## 🚀 System Highlights & Features

*We built this system from the ground up, from the mechanical CAD design to the physical manufacturing and physics engine.*

### 1. Fusion 360 CAD Design
![Fusion 360 Assembly](Images/Prototype.png)
*The complete mechanical assembly designed in Autodesk Fusion 360, detailing the 10-disk backbone, exact 5.0 mm tendon routing channels, and the actuation base prior to physical manufacturing.*

### 2. The Physical Hardware Prototype
To ensure mechanical stability and precision, we manufactured a physical 10-disk prototype utilizing 4 stainless steel rods. This specific design choice successfully prevents torsional deformation (twisting) during operation, supported by compressive springs and actuated by NEMA-17 stepper motors. To fully replicate a true clinical endoscope, we integrated a micro-camera directly at the distal tip of the continuum backbone to provide real-time visual feedback to the operator.

### 3. ROS 2 & Gazebo Digital Twin (Anatomical Navigation)
We developed a highly accurate digital twin in Gazebo Harmonic, driven by our custom Inverse Kinematics solver. To rigorously test the system, we constructed a custom simulated environment by importing 3D meshes of a human trachea and bronchial tree. We inserted our digital robot into this anatomy to perform complex clinical maneuvers, mimicking reality and validating collision dynamics. Additionally, a simulated camera sensor is attached to the tip of the digital twin, streaming real-time video that perfectly matches the physical hardware's perspective.

### 4. Interactive Operator Dashboard
![UI Dashboard](Images/Heidi%20hand.jpeg)
*Our custom Tkinter GUI calculating discrete Constant Curvature tendon lengths (L1, L2, L3, L4) from Task-Space (XYZ) inputs.*

### 5. RViz 2 Holographic Tracking
To validate our control mathematics in real-time, we developed a visualization node using RViz 2. It utilizes TF2 coordinate transformations to render a real-time holographic projection of the continuum spline and tendon routing (5mm radius), ensuring the software's geometric calculations perfectly align with the hardware's physical state.

---

## 📄 Research Paper
As part of this project, our team authored a comprehensive research paper detailing our mathematical modeling, anti-twist hardware design, and simulation architecture. 

**You can read the full documentation and findings in the [`Paper/`](./Paper) folder of this repository.**

---

## System Architecture

### 🛠️ Hardware (Physical Prototype)
Our team fully designed and manufactured the physical robot to validate our simulation math:
* **Backbone:** 10-Disk flexible structure utilizing 4 stainless steel rods to prevent twisting and maintain structural integrity, combined with compressive springs.
* **Physical Dimensions:** Strictly 230.0 mm total resting length.
* **Endoscopic Vision:** A micro-camera is mounted directly at the distal tip of the continuum backbone.
* **Actuation:** 4-Tendon antagonistic routing system (exactly 5.0 mm routing radius) driven by high-precision NEMA-17 stepper motors.
* **Control Base:** Arduino-based microcontrollers handling low-level motor pulses, translating our Python solver's commands into physical tension.

### 💻 Software (Digital Twin & Control)
* **Anatomical Environment:** A full physics simulation in Gazebo Harmonic featuring realistic human trachea and bronchi meshes for environmental collision and maneuver testing.
* **Simulated Endoscopic Vision:** A virtual camera sensor on the tip of the digital twin.
* **Kinematics Solver:** A custom Python ROS 2 node implementing discrete Constant Curvature Inverse/Forward Kinematics to calculate exact tendon lengths and joint angles, preventing standard continuous-arc scaling errors.
* **Interactive Dashboard:** A GUI for real-time task-space (XYZ) and joint-space operator control.
* **Visualizer:** A custom node drawing the physical tendons and stainless steel rods perfectly anchored to the base in RViz 2.

---

## Installation & Setup

### Prerequisites
* Ubuntu 24.04
* ROS 2 Jazzy
* Gazebo Harmonic

### Building the Workspace
We recommend performing a clean build of the package to prevent any caching issues with the URDF or custom Python nodes.

```bash
# 1. Create the workspace and clone the repository
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone [https://github.com/Ahmed-Etman-111/tendon-driven-continuum-robot.git](https://github.com/Ahmed-Etman-111/tendon-driven-continuum-robot.git)

# 2. Navigate to the workspace root
cd ~/ros2_ws

# 3. Clean previous builds for this specific package
rm -rf build/tendon_continuum install/tendon_continuum

# 4. Build the package and source the overlay
colcon build --packages-select tendon_continuum
source install/setup.bash
```

---

## Launching the System
Once the workspace is built and sourced, launch the entire digital twin—including Gazebo, RViz 2, the custom TF broadcasters, and the Tkinter operator dashboard—with a single command:

```bash
ros2 launch tendon_continuum sim.launch.py
```

---

## Mathematical Foundation
Our Inverse Kinematics solver avoids standard rigid-link approximations in favor of exact discrete arc geometry based on established continuum robotics research. The tendon lengths ($L_1, L_2, L_3, L_4$) are calculated dynamically to account for the 10-disk physical configuration and the exact 5.0 mm routing offset, ensuring 1-to-1 parity between the digital commands and the physical hardware.
