# Robotics I — RAS 545

### Arizona State University — Spring 2025  
**Instructor:** Dr. Mostafa Yourdkhani  
**Student:** Chih-Hao (Andy) Tsai  

---

## Overview
This repository presents my complete coursework for **RAS 545: Robotics Systems I** at Arizona State University,  
covering both **MATLAB-based kinematics/control analysis** and **ROS 2–based robotic system integration**.  

The course is divided into two main components:  
1. **Homework Assignments (MATLAB & Simulink):** Analytical modeling, kinematics, dynamics, and control design.  
2. **Laboratories (ROS 2 & MyCobot Pro 600):** Practical implementation of robot control, perception, and path planning in simulation and real hardware.

---

## Table of Contents
- [Technical Overview](#technical-overview)
- [Homework Series — Robotics and Control Systems](#homework-series--robotics-and-control-systems)
- [Laboratory Series — Vision-Based Path Planning](#laboratory-series--vision-based-path-planning)
- [Final Integration — ROS 2 Maze Solving Project](#final-integration--ros-2-maze-solving-project)
- [Skills & Tools Summary](#skills--tools-summary)

---

## Technical Overview
This course builds the foundation of modern robotic systems by combining **mathematical modeling**, **control theory**, and **ROS-based implementation**.

| Domain | Focus | Tools |
|--------|--------|-------|
| Kinematics | Forward/Inverse Kinematics, DH Parameters | MATLAB |
| Dynamics | Lagrangian Modeling, Equation of Motion | MATLAB / Simulink |
| Control | PID, Feedback Stability | MATLAB |
| Simulation | ROS 2 + Gazebo + RViz2 | Python / C++ |
| Vision & Planning | OpenCV, A* Path Search | Python |
| Hardware | MyCobot Pro 600, TCP/IP Control | ROS 2 |

---

## Homework Series — Robotics and Control Systems
**Repository:** [`/HWs`](./HWs)  
**Language/Tools:** MATLAB, Simulink  

### Topics Covered
- **Forward and Inverse Kinematics:** Multi-DOF and SCARA robots  
- **Dynamic Modeling:** Crank–slider mechanism using Lagrangian formulation  
- **PID Control Design:** Closed-loop response analysis and tuning  
- **Simulation:** MATLAB/Simulink model validation and comparison  

📘 *Core Concepts:* DH parameters · Euler–Lagrange formulation · PID tuning  
📂 *Read full README:* [`HWs/README.md`](./HWs/README.md)

---

## Laboratory Series — Vision-Based Path Planning
**Repository:** [`/Labs`](./Labs)  
**Frameworks:** ROS 2 Humble, Gazebo, RViz2  
**Robot:** MyCobot Pro 600 (6-DOF)

### Topics Covered
- **Digital Twin Setup:** URDF modeling and Gazebo simulation  
- **ROS 2 Node Development:** rclpy publishers/subscribers for motion control  
- **Computer Vision Pipeline:** OpenCV-based maze detection, skeletonization, and coordinate mapping  
- **Path Planning:** A* search, graph generation, and trajectory interpolation  
- **Real-Time Control:** TCP/IP communication with MyCobot Pro 600  

📘 *Core Concepts:* ROS 2 topics · OpenCV · Path interpolation · Real-world validation  
📂 *Read full README:* [`Labs/README.md`](./Labs/README.md)

---

## Final Integration — ROS 2 Maze Solving Project
**Title:** *Vision-Based Path Planning with MyCobot Pro 600*  
**Summary:**  
Integrated all modules (perception, kinematics, and control) into a unified ROS 2 node for **autonomous maze navigation**.  
The system captures maze images, extracts paths, computes the shortest route using **A\*** search, and executes motion commands in **real time**.  
Validated both in **Gazebo** and **real robot experiments**.

📂 *Report:* [`Labs/reports/final_project.pdf`](./Labs/reports/final_project.pdf)

---

## Skills & Tools Summary

| Category | Skills / Tools |
|-----------|----------------|
| **Modeling & Control** | MATLAB · Simulink · Symbolic Math Toolbox · Control System Toolbox |
| **Kinematics & Dynamics** | Forward/Inverse Kinematics · DH Table · Lagrangian Mechanics |
| **ROS 2 & Simulation** | ROS 2 Humble · Gazebo · RViz2 · tf2 · URDF modeling |
| **Computer Vision** | OpenCV · Image Skeletonization · Graph-based Path Search |
| **Programming** | Python · C++ · MATLAB |
| **Hardware Integration** | MyCobot Pro 600 · TCP/IP control · Real-robot motion execution |

