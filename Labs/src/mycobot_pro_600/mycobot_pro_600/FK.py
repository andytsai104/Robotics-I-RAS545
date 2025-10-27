# This file conduct forward kinematics for Cobot Pro 600 Simulation
import rclpy
from pymycobot.elephantrobot import ElephantRobot
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import sys
import numpy as np


class ForwardKinematics_Subscriber(Node):
    def __init__(self):
        super().__init__("FK")
        self.subscription = self.create_subscription(
            JointState,
            "joint_states",
            self.joint_state_callback,
            10
        )

    def joint_state_callback(self, msg):
        joint_angles = msg.position
        # FK math here — dummy output for now
        x, y, z = self.forward_kinematics(joint_angles)
        self.get_logger().info(f'End-effector position: x={x:.2f}, y={y:.2f}, z={z:.2f}')

    def forward_kinematics(self, joint_angles):
        dh_table = np.array([
            [joint_angles[0],         219.34,   0.0,         0],
            [-np.pi/2+joint_angles[1],    0.0,   0.0,   -np.pi/2],
            [joint_angles[2],            0.0, 250.0,         0],
            [-np.pi/2+joint_angles[3],  109.1, 250.0,         0],
            [joint_angles[4],            108.0,  0.0,   -np.pi/2],
            [joint_angles[5],          75.86,   0.0,    np.pi/2]
        ])

        T = np.eye(4)
        
        for i in range(len(dh_table)):
            theta, d, a, alpha = dh_table[i]
            T = T @ self.form_T(theta, d, a, alpha)

        x, y, z = T[0, 3], T[1, 3], T[2, 3]
        return x, y, z
    
    def form_T(self, theta, d, a, alpha):
        # T = np.array([[np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        #               [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        #               [0, np.sin(alpha), np.cos(alpha), d],
        #               [0, 0, 0, 1]])
        # return T

        return np.array([
            [np.cos(theta), -np.sin(theta), 0, a],
            [np.sin(theta)*np.cos(alpha), np.cos(theta)*np.cos(alpha), -np.sin(alpha), -d*np.sin(alpha)],
            [np.sin(theta)*np.sin(alpha), np.cos(theta)*np.sin(alpha),  np.cos(alpha),  d*np.cos(alpha)],
            [0, 0, 0, 1]
        ])



def main(args=None):
    rclpy.init(args=args)
    FK_subscriber = ForwardKinematics_Subscriber()

    rclpy.spin(FK_subscriber)

    FK_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()