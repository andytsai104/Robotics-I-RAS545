# Robotics and Control Systems Coursework

This repository contains homework assignments and reports completed for a **Robotics and Control Systems** course, primarily utilizing **MATLAB** and **Simulink** for analysis, simulation, and modeling.

---

## 📑 Table of Contents
1. [Homework 1 - Foundational Training (Certificates)](#foundational-training)
2. [Homework 2 — Industrial Robot Kinematics](#hw2)
3. [Homework 3 — SCARA Robot Kinematics & Simulation](#hw3)
4. [Homework 4 — Dynamic Analysis of a Crank-Slider Mechanism](#hw4)
5. [Homework 5 — PID Control System Design](#hw5)

---
## Homework 1 - Foundational Training (Certificates)

These certificates represent foundational skills in MATLAB used throughout the coursework.

- ![MATLAB Fundamentals](./HW/HW1/fund.png)
- [cite_start]**MATLAB Programming Techniques** [cite: 573]
- [cite_start]**MATLAB for Data Processing and Visualization** [cite: 5]
- [cite_start]**Solving Ordinary Differential Equations with MATLAB** [cite: 587]

---
## Homework 2 — Industrial Robot Kinematics {#hw2}

- [cite_start]Performed **Forward Kinematics** for a multi-DOF industrial robot[cite: 591].
- [cite_start]Calculated the homogeneous transformation matrices ($^{i}T_{i+1}$) using the **Projection Matrix** method[cite: 595, 596].
- [cite_start]Verified the final end-effector position and homogeneous transformation using **MATLAB**[cite: 627, 631].
- [cite_start]Also presented a Denavit-Hartenberg (DH) table analysis for the same robot[cite: 751].
📂 [Report](./HW2/HW2.pdf)

---
## Homework 3 — SCARA Robot Kinematics & Simulation {#hw3}

- [cite_start]Derived the **Forward Kinematics** for a SCARA robot, resulting in the final homogeneous transformation matrix $^0T_{E.E.}$[cite: 246, 247].
- [cite_start]Used a custom MATLAB function to verify the derived transformation matrix and end-effector position[cite: 256, 300].
- [cite_start]Established the robot's **workspace** (maximum reach $0.85\text{ m}$ in the $x$-$y$ plane and $0.6\text{ m}$ in $z$)[cite: 317, 375].
- [cite_start]Implemented and verified the Forward Kinematics model in **Simulink**[cite: 435, 467].
- [cite_start]Calculated **Inverse Kinematics** in MATLAB and used the results to draw a square path in the Simulink simulator[cite: 473, 527].
📂 [Report](./HW3/HW3.pdf)

---
## Homework 4 — Dynamic Analysis of a Crank-Slider Mechanism {#hw4}

- [cite_start]Developed the kinematic model of a **Crank-Slider Mechanism**[cite: 135].
- [cite_start]Calculated the rotation angle $\beta$ of link 2 and the position of the three masses ($m_1$, $m_2$, $m_3$)[cite: 156, 157].
- [cite_start]Formulated the system's **Potential Energy ($P_{\text{tot}}$)** and **Kinetic Energy ($K_{\text{tot}}$)**[cite: 159, 161].
- [cite_start]Determined the **Lagrangian of the system ($L$)**[cite: 169, 170].
- [cite_start]Derived the **Equation of Motion** using the Euler-Lagrange method[cite: 173, 175].
- [cite_start]Simulated the system's response (crank angle $\theta(t)$ and slider position $x_C(t)$) in **MATLAB/Simulink**[cite: 178, 185, 201].
📂 [Report](./HW4/HW4.pdf)

---
## Homework 5 — PID Control System Design {#hw5}

- [cite_start]Defined the **Plant Transfer Function** for the system[cite: 13].
- [cite_start]Analyzed the **Open-loop system response** to a sinusoidal input $r(t)=5\sin(2t)$, noting the lack of tracking and large amplitude/phase lag[cite: 15, 33].
- [cite_start]Designed a **Closed-loop control system** using a PID controller[cite: 34].
- Justified the selection of PID parameters by testing and comparing three configurations:
    * [cite_start]**Proportional ($K_p=1$):** Quick response but large steady-state error and persistent oscillations[cite: 52].
    * [cite_start]**Proportional-Integral (PI: $K_p=0.5, K_i=3$):** Reduced steady-state error, but became unstable due to integrator windup[cite: 54].
    * [cite_start]**Proportional-Integral-Derivative (PID: $K_p=0.5, K_i=5, K_d=15$):** Achieved the best result with close tracking, low error, and minimal delay[cite: 55, 56].
- [cite_start]Provided a comparison plot illustrating the significant improvement in tracking performance with the final PID controller[cite: 119, 129].
📂 [Report](./HW5/HW5.pdf)
