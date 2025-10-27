import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from scipy.optimize import minimize
import numpy as np


class SetHome(Node):
    def __init__(self):
        super().__init__('Set_Home')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.home = np.array([
            [-397.942, -158.419],     # top-left
            [-222.111, -197.865],   # top-right
            [-369.510, -343.600],    # bottom-left
            [-227.516, -338.718]    # bottom-right
        ], dtype=np.float32)
        self.x, self.y, self.z = self.home[0][0], self.home[0][1], 160
        self.desired_position = np.array([self.x, self.y, self.z])
        self.timer = self.set_home()

    def set_home(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        theta1, theta2, theta3, theta4, theta5, theta6 = self.inverse_kinematics(self.x, self.y, self.z)
        msg.position = [theta1, theta2, theta3, theta4, theta5, theta6]
        self.publisher.publish(msg)
        self.get_logger().info(f"IK: θ1={theta1:.2f}, θ2={theta2:.2f}, θ3={theta3:.2f}, θ4={theta4:.2f}, θ5={theta5:.2f}, θ6={theta6:.2f}")
        self.get_logger().info(f"Published for home position: ({self.x:.2f}, {self.y:.2f}, {self.z:.2f})")

    
    def forward_kinematics(self, q):
        # th1, th2, th3, th4, th5, th6 = q
        th1, th2, th3, th6 = q
        th4 = th4 = -np.pi/2 - (th2 + th3)
        th5 = np.pi/2
            
        # Position components (simplified notation)
        Px = (
        250 * np.cos(th1) * np.sin(th2)
        - (1091 * np.sin(th1)) / 10
        - (3793 * np.cos(th5) * np.sin(th1)) / 50
        + 250 * np.cos(th1) * np.cos(th2) * np.sin(th3)
        + 250 * np.cos(th1) * np.cos(th3) * np.sin(th2)
        + 108 * np.cos(th1) * np.cos(th2) * np.cos(th3) * np.sin(th4)
        + 108 * np.cos(th1) * np.cos(th2) * np.cos(th4) * np.sin(th3)
        + 108 * np.cos(th1) * np.cos(th3) * np.cos(th4) * np.sin(th2)
        - 108 * np.cos(th1) * np.sin(th2) * np.sin(th3) * np.sin(th4)
        - (3793 * np.cos(th1) * np.cos(th2) * np.cos(th3) * np.cos(th4) * np.sin(th5)) / 50
        + (3793 * np.cos(th1) * np.cos(th2) * np.sin(th3) * np.sin(th4) * np.sin(th5)) / 50
        + (3793 * np.cos(th1) * np.cos(th3) * np.sin(th2) * np.sin(th4) * np.sin(th5)) / 50
        + (3793 * np.cos(th1) * np.cos(th4) * np.sin(th2) * np.sin(th3) * np.sin(th5)) / 50
        )

        Py = (
            (1091 * np.cos(th1)) / 10
            + (3793 * np.cos(th1) * np.cos(th5)) / 50
            + 250 * np.sin(th1) * np.sin(th2)
            + 250 * np.cos(th2) * np.sin(th1) * np.sin(th3)
            + 250 * np.cos(th3) * np.sin(th1) * np.sin(th2)
            + 108 * np.cos(th2) * np.cos(th3) * np.sin(th1) * np.sin(th4)
            + 108 * np.cos(th2) * np.cos(th4) * np.sin(th1) * np.sin(th3)
            + 108 * np.cos(th3) * np.cos(th4) * np.sin(th1) * np.sin(th2)
            - 108 * np.sin(th1) * np.sin(th2) * np.sin(th3) * np.sin(th4)
            - (3793 * np.cos(th2) * np.cos(th3) * np.cos(th4) * np.sin(th1) * np.sin(th5)) / 50
            + (3793 * np.cos(th2) * np.sin(th1) * np.sin(th3) * np.sin(th4) * np.sin(th5)) / 50
            + (3793 * np.cos(th3) * np.sin(th1) * np.sin(th2) * np.sin(th4) * np.sin(th5)) / 50
            + (3793 * np.cos(th4) * np.sin(th1) * np.sin(th2) * np.sin(th3) * np.sin(th5)) / 50
        )

        Pz = (
            108 * np.cos(th2 + th3 + th4)
            - (3793 * np.cos(th2 + th3 + th4 + th5)) / 100
            + 250 * np.cos(th2 + th3)
            + 250 * np.cos(th2)
            + (3793 * np.cos(th2 + th3 + th4 - th5)) / 100
            + 10967 / 50
        )

        return np.array([Px, Py, Pz])

    def inverse_kinematics(self, x, y, z):
        def error_function(q):
            th1, th2, th3, th6 = q
            th4 = -np.pi/2 - (th2 + th3)
            th5 = np.pi/2
            full_q = [th1, th2, th3, th6]
            current_position = self.forward_kinematics(full_q)
            return np.sum((current_position - self.desired_position)**2)  # Squared error
        initial_guess = np.array([0.0, 0.0, 0.0, 0.0])
        # bounds = [
        #     (-np.pi, np.pi),     # q1
        #     (-np.pi/2, np.pi/2), # q2 
        #     (-np.pi, np.pi), # q3
        #     # (-np.pi/2, np.pi/2), # q4
        #     # (-np.pi/2, np.pi/2), # q5
        #     (-np.pi, np.pi)      # q6
        # ]
        bounds = [
            (-np.pi, np.pi),            # q1
            (-np.pi/2, 0),              # q2 
            (-np.pi+(np.pi-2.5), 2/3*np.pi),# q3
            # (-np.pi/2, np.pi/2),      # q4
            # (-np.pi/2, np.pi/2),      # q5
            (-np.pi, np.pi)             # q6
        ]

        # Optimization options
        options = {
            'maxiter': 1000,
            'ftol': 1e-6,
            'eps': 1e-8
        }

        # Try multiple optimization methods
        for method in ['L-BFGS-B', 'SLSQP', 'TNC']:
            print(f"\nTrying method: {method}")
            result = minimize(
                error_function,
                initial_guess,
                method=method,
                bounds=bounds,
                options=options
            )
            if result.success:
                # return result.x
                th1, th2, th3, th6 = result.x
                th4 = -np.pi/2 - (th2 + th3)
                th5 = np.pi/2
                return [th1, th2, th3, th4, th5, th6]
            else:
                print(f"Method {method} failed:", result.message)
        else:
            result = minimize(
                self.error_function,
                initial_guess,
                method='L-BFGS-B',
                bounds=bounds,
                options={'ftol': 1e-3, 'maxiter': 5000}
            )
            if result.success:
                th1, th2, th3, th6 = result.x
                th4 = -np.pi/2 - (th2 + th3)
                th5 = np.pi/2
                return [th1, th2, th3, th4, th5, th6]
            else:
                print(f"Method {method} failed:", result.message)


def main(args=None):
    rclpy.init()
    node = SetHome()
    node.destroy_node()
    rclpy.shutdown()