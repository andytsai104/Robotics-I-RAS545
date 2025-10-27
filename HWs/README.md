# Robotics and Control Systems Coursework

This repository contains homework assignments and reports for the **Robotics and Control Systems** course, using **MATLAB** and **Simulink** for analysis, simulation, and modeling.

---

## 📑 Table of Contents
1. [Homework 1 – Foundational Training](#hw1)
2. [Homework 2 – Industrial Robot Kinematics](#hw2)
3. [Homework 3 – SCARA Robot Kinematics & Simulation](#hw3)
4. [Homework 4 – Crank-Slider Mechanism Dynamics](#hw4)
5. [Homework 5 – PID Control Design](#hw5)

---

## Homework 1 – Foundational Training {#hw1}
Certificates demonstrating MATLAB fundamentals used in later assignments:
- ![MATLAB Fundamentals](./HW1/fund.png)
- ![Programming Techniques](./HW1/PT.png)
- ![Data Processing & Visualization](./HW1/Vi.png)
- ![ODEs with MATLAB](./HW1/ODE.png)

---

## Homework 2 – Industrial Robot Kinematics {#hw2}
- Performed **forward kinematics** for a multi-DOF robot.  
- Computed homogeneous transformation matrices ($^{i}T_{i+1}$) using the **projection matrix** method.  
- Verified end-effector position in **MATLAB**.  
- Constructed a **DH parameter table** for the same robot.  
📂 [Report](./HW2/HW2.pdf)

---

## Homework 3 – SCARA Robot Kinematics & Simulation {#hw3}
- Derived **forward kinematics** and obtained $^0T_{E.E.}$.  
- Verified results using a custom MATLAB function.  
- Determined the **workspace** (reach: 0.85 m in $x$–$y$, 0.6 m in $z$).  
- Implemented the model in **Simulink**.  
- Solved **inverse kinematics** and simulated a square trajectory.  
📂 [Report](./HW3/HW3.pdf)

---

## Homework 4 – Crank-Slider Mechanism Dynamics {#hw4}
- Modeled the **crank-slider mechanism**.  
- Calculated link angles and mass positions.  
- Derived **potential**, **kinetic**, and **Lagrangian** equations.  
- Obtained the **equation of motion** using Euler–Lagrange formulation.  
- Simulated $	heta(t)$ and $x_C(t)$ responses in **MATLAB/Simulink**.  
📂 [Report](./HW4/HW4.pdf)

---

## Homework 5 – PID Control Design {#hw5}
- Defined the **plant transfer function**.  
- Analyzed the **open-loop** response to $r(t)=5\sin(2t)$.  
- Designed a **PID controller** and compared control strategies:  
  - **P ($K_p=1$):** Fast but large steady-state error.  
  - **PI ($K_p=0.5, K_i=3$):** Reduced error, unstable due to windup.  
  - **PID ($K_p=0.5, K_i=5, K_d=15$):** Best tracking and stability.  
- Included comparison plots showing improved tracking performance.  
📂 [Report](./HW5/HW5.pdf)
