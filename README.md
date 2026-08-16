# Smart Endoscopic Autonomous System: One-Port Continuum Surgical Robot

[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange.svg)](https://gazebosim.org/)
[![Python](https://img.shields.io/badge/Python-3.12-blue.svg)](https://www.python.org/)

## Overview
This repository contains the software simulation and control architecture for a one-port tendon-driven continuum surgical robot. Developed as a graduation project by Team Four from the Systems and Biomedical Engineering department at Cairo University, this cyber-physical system bridges a physical nitinol-core prototype with a highly accurate digital twin.

**Funding & Support:** This project was developed with support from ITIDA's ITAC funding program (2025–2026).

![System Overview](images/your_canva_slide_image.png) *<!-- Replace with your slide image -->*

## System Architecture

The project is split into two perfectly synchronized domains:

### 1. Hardware (Physical Prototype)
* **Backbone:** 10-Disk flexible structure with a super-elastic Nitinol core and compressive springs (230 mm total length).
* **Actuation:** 4-Tendon antagonistic routing system (5 mm routing radius) driven by NEMA-17 stepper motors.
* **Control Base:** Arduino-based microcontrollers handling low-level motor pulses.

### 2. Software (Digital Twin & Control)
* **Digital Twin:** A full physics simulation in Gazebo Harmonic featuring collision detection for anatomical environments (e.g., trachea meshes).
* **Kinematics Solver:** A custom Python ROS 2 node implementing discrete Constant Curvature Inverse/Forward Kinematics to calculate exact tendon lengths and joint angles.
* **Interactive Dashboard:** A Tkinter-based GUI for real-time task-space (XYZ) and joint-space operator control.
* **RViz 2 Validation:** Real-time holographic rendering of the continuum spline using TF2 coordinate transformations.

## Installation & Setup

### Prerequisites
* Ubuntu 24.04
* ROS 2 Jazzy
* Gazebo Harmonic

### Building the Workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone [https://github.com/yourusername/smart-endoscopic-system.git](https://github.com/yourusername/smart-endoscopic-system.git)
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
