# Vision-Based Path Planning with MyCobot Pro 600 (ROS 2)

This repository contains all ROS 2 packages and reports for a series of labs culminating in a complete
image-based path planning system for the **MyCobot Pro 600** robot.  
The pipeline progresses from basic kinematics to autonomous maze solving.

---

## 📑 Table of Contents
1. [Lab 1 — Forward Kinematics of Dobot Magician Lite](#lab-1)
2. [Lab 2 — Digital Twin Development with ROS 2](#lab-2)
3. [Lab 3 — Vision-Based Path Planning](#lab-3)
4. [Lab 4 — Curved Path Following](#lab-4)
5. [Final Project — Maze Solving via A* Search](#final-project)

---

## Lab 1 — Forward Kinematics of Dobot Magician Lite
- Built and analyzed the **kinematic chain** using MATLAB.
- Verified forward kinematics via homogeneous transformation matrices.
- Learned the use of **effective joint angles** for realistic motion modeling.  
📂 [Report](./reports/Group7_Lab1.pdf)

---

## Lab 2 — Digital Twin (ROS 2 + Gazebo)
- Created a **URDF model** of the Dobot Magician Lite.
- Set up **ROS 2 workspace**, simulated robot in **RViz2 / Gazebo**.
- Implemented **forward & inverse kinematics** verification between MATLAB and simulation.  
📂 [Report](./reports/Group7_Lab2.pdf)

---

## Lab 3 — Vision-Based Path Planning
- Transitioned to **MyCobot Pro 600**.
- Built an image-to-world mapping pipeline using **OpenCV** + skeletonization.
- Implemented a **custom inverse kinematics solver** with constraint θ₂ + θ₃ + θ₄ = −π⁄2.  
- Controlled both simulation and real robot via **ROS 2 topics** and **TCP/IP**.  
📂 [Report](./reports/Group7_Lab3.pdf)

---

## Lab 4 — Curved Path Following
- Extended Lab 3 pipeline for **smooth curved trajectories**.
- Used **path interpolation** and **timed ROS 2 publishing** for continuous motion.
- Validated downward orientation and kinematic accuracy.  
📂 [Report](./reports/Group7_Lab4.pdf)

---

## Final Project — Smart and Accurate Maze Solving
- Integrated **A\* search** to autonomously compute the **shortest maze path**.
- Applied **morphological dilation** and **distance-transform cost maps** for safety margins.
- Wrapped perception + planning + control into a single **ROS 2 node** (`maze_solver_publisher.py`).
- Robot successfully executed maze traversal in both RViz and the physical setup.  
📂 [Report](./reports/final_project.pdf)

---

## ⚙️ Technical Highlights
- **Framework:** ROS 2 Humble + Gazebo + RViz2  
- **Languages:** Python 3 & C++  
- **Libraries:** OpenCV, NumPy, tf2, rclpy  
- **Hardware:** MyCobot Pro 600 (6-DOF)  
- **Kinematics:** Custom FK/IK solver with θ₂ + θ₃ + θ₄ = −π⁄2 constraint  

---

## 📂 ROS 2 Workspace
The `src/` directory includes all packages used across the labs.  
Each lab’s version builds upon the previous one — the final package (`maze_solver`) integrates all prior work.

To build:
```bash
cd ~/mycobot_path_planning_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch maze_solver maze_solver.launch.py

