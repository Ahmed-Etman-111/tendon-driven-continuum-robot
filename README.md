# Smart Endoscopic Autonomous System: One-Port Continuum Surgical Robot

[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange.svg)](https://gazebosim.org/)
[![Python](https://img.shields.io/badge/Python-3.12-blue.svg)](https://www.python.org/)

## Overview
This repository contains the software simulation, mathematical kinematics solver, and hardware control architecture for a one-port tendon-driven continuum surgical robot. Developed as a graduation project by Team Four from the Systems and Biomedical Engineering department at Cairo University, this cyber-physical system bridges a physical nitinol-core prototype with a highly accurate digital twin.

**Funding & Support:** This project was developed with support from ITIDA's ITAC funding program (2025–2026).

# System Overview
<p align="center">
  <img src="Images/gazebo_simulation_gui.png" width="45%" title="System Overview" />
  <img src="Images/IMG_20260627_190525.jpeg" width="45%" title="Hardware Prototype" />
</p> 

---

## 🎥 Visual Demonstrations

*We built this system from the ground up, from the mechanical CAD design to the physical manufacturing and physics engine.*

### 1. Fusion 360 CAD Design
![Fusion 360 Assembly](Images/Prototype.png)
*The complete mechanical assembly designed in Autodesk Fusion 360, detailing the 10-disk backbone, exact 5.0 mm tendon routing channels, and the actuation base prior to physical manufacturing.*

### 2. The Physical Hardware Prototype
![Hardware Demo](images/hardware_prototype_bending.gif)
*Our physical 10-disk prototype bending. The system uses a super-elastic Nitinol core and compressive springs, actuated by NEMA-17 stepper motors.*

### 3. ROS 2 & Gazebo Digital Twin
![Gazebo Simulation](images/gazebo_simulation.gif)
*Real-time simulation in Gazebo Harmonic, driven by our custom Inverse Kinematics solver. The digital twin bends identically to the physical hardware.*

### 4. Interactive Operator Dashboard
![UI Dashboard](Images/Heidi%20hand.jpeg)
*Our custom Tkinter GUI calculating discrete Constant Curvature tendon lengths (L1, L2, L3, L4) from Task-Space (XYZ) inputs.*

### 5. RViz 2 Holographic Tracking
![RViz TF Tracking](images/rviz_tracking.gif)
*Real-time validation of the continuum spline and tendon routing (5mm radius) using TF2 coordinate transformations.*

---

## System Architecture

### 🛠️ Hardware (Physical Prototype)
Our team fully designed and manufactured the physical robot to validate our simulation math:
* **Backbone:** 10-Disk flexible structure with a super-elastic Nitinol core and stainless steel compressive springs.
* **Physical Dimensions:** Strictly 230.0 mm total resting length.
* **Actuation:** 4-Tendon antagonistic routing system (exactly 5.0 mm routing radius) driven by high-precision NEMA-17 stepper motors.
* **Control Base:** Arduino-based microcontrollers handling low-level motor pulses, translating our Python solver's commands into physical tension.

### 💻 Software (Digital Twin & Control)
* **Digital Twin:** A full physics simulation in Gazebo Harmonic featuring collision detection for anatomical environments.
* **Kinematics Solver:** A custom Python ROS 2 node implementing discrete Constant Curvature Inverse/Forward Kinematics to calculate exact tendon lengths and joint angles, preventing standard continuous-arc scaling errors.
* **Interactive Dashboard:** A GUI for real-time task-space (XYZ) and joint-space operator control.
* **Visualizer:** A custom node drawing the physical tendons and rods perfectly anchored to the base in RViz 2.

---

## Installation & Setup

### Prerequisites
* Ubuntu 24.04
* ROS 2 Jazzy
* Gazebo Harmonic

### Building the Workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# Clone this repository
git clone https://github.com/Ahmed-Etman-111/tendon-driven-continuum-robot.git
cd ~/ros2_ws
# Build the package
colcon build --symlink-install
source install/setup.bash
