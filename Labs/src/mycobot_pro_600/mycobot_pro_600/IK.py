import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
from scipy.optimize import minimize
from math import atan2, sqrt


class IKPublisher(Node):
    def __init__(self):
        super().__init__('IK')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_state)

        # Target end-effector location
        self.x = 372.113
        self.y =  -178.3
        self.z = 70
        self.desired_position = np.array([self.x, self.y, self.z])

        # Solve IK once
        try:
            self.theta1, self.theta2, self.theta3, self.theta4, self.theta5, self.theta6 = self.inverse_kinematics(self.x, self.y, self.z)
            self.get_logger().info(f"IK: θ1={self.theta1:.2f}, θ2={self.theta2:.2f}, θ3={self.theta1:.2f}, θ4={self.theta1:.2f}, θ5={self.theta1:.2f}, θ6={self.theta1:.2f}")
        except ValueError as e:
            self.get_logger().error(str(e))
            rclpy.shutdown()
    def forward_kinematics(self, q):
        th1, th2, th3, th4, th5, th6 = q
        
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
            current_position = self.forward_kinematics(q)
            return np.sum((current_position - self.desired_position)**2)  # Squared error
        initial_guess = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        bounds = [
            (-np.pi, np.pi),    # q1
            (-np.pi/2, np.pi/2), # q2 
            (-np.pi/2, np.pi/2), # q3
            (-np.pi/2, np.pi/2), # q4
            (-np.pi/2, np.pi/2), # q5
            (-np.pi, np.pi)      # q6
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
                return result.x
            else:
                print(f"Method {method} failed:", result.message)
        else:
            # print("All optimization methods failed. Trying with relaxed tolerances...")
            result = minimize(
                self.error_function,
                initial_guess,
                method='L-BFGS-B',
                bounds=bounds,
                options={'ftol': 1e-3, 'maxiter': 5000}
            )
            if result.success:
                return result.x
            else:
                print(f"Method {method} failed:", result.message)

    def publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        msg.position = [self.theta1, self.theta2, self.theta3, self.theta4, self.theta5, self.theta6]
        self.publisher.publish(msg)


    def reset_robot(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        msg.position = [0.0] * 6
        self.publisher.publish(msg)
        self.get_logger().info("Reset joints to zero.")

def main(args=None):
    rclpy.init(args=args)
    node = IKPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.reset_robot_pose()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
