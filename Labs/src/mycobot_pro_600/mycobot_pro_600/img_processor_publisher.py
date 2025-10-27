import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import numpy as np
from scipy.optimize import minimize
import cv2
from scipy.interpolate import splprep, splev
import os

class ImgProcessPublisher(Node):
    def __init__(self):
        super().__init__('ImgProcessor')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(2.5, self.publish_next_joint_state)

        # define home position coordinates
        self.x, self.y, self.z = -372.113, -178.3, 70
        self.path_index = 0
        self.image_crop, self.path_pts, self.path_world_coords = self.get_path_pts()
        self.show_target_pth(self.image_crop, self.path_pts, self.path_world_coords)

        if self.path_world_coords is None:
            self.get_logger().error("No path points found.")
            rclpy.shutdown()
            return

    def skeletonize(self, img):
        if len(img.shape) == 3:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # Convert to binary
        _, img = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)
        img = img // 255

        skel = np.zeros(img.shape, np.uint8)
        kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (3,3))

        while True:
            eroded = cv2.erode(img, kernel)
            temp = cv2.dilate(eroded, kernel)
            temp = cv2.subtract(img, temp)
            skel = cv2.bitwise_or(skel, temp)
            img = eroded.copy()
            if cv2.countNonZero(img) == 0:
                break

        skel = skel * 255  # scale back to [0,255]
        return skel


    def get_path_pts(self):
        # cap = cv2.VideoCapture(2)
        # ret, frame = cap.read()
        # cap.release()
        pkg_pth = '~/ras545_ws/src/mycobot_pro_600'
        start_y, start_x, end_y, end_x = 55, 150, 400, 470
        # img_pth = pkg_pth+'/sample_imgs/real_img.jpg'
        # cv2.imwrite(img_pth, frame)
        curve = False
        if curve:
            img_pth = os.path.expanduser(pkg_pth+'/sample_imgs/L4_sample.png')
            start_y, start_x, end_y, end_x = 0, 0, 589, 587
        else:
            img_pth = os.path.expanduser(pkg_pth+'/sample_imgs/captured_image.jpg')
            # img_pth = os.path.expanduser(pkg_pth+'/sample_imgs/sample_img.jpeg')
        # img = frame
        img = cv2.imread(img_pth)
        image_crop = img[start_y:end_y, start_x:end_x]
        # cv2.imshow('image',image_crop)
        # cv2.waitKey(0)

        gray = cv2.cvtColor(image_crop, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (15, 15), 0)
        _, binary = cv2.threshold(blurred, 100, 255, cv2.THRESH_BINARY_INV)

        # Skeletonize
        skeleton = self.skeletonize(binary)

        # Contour detection
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        cnt = max(contours, key=cv2.contourArea)  # use the largest contour

        # Create contour mask
        mask = np.zeros_like(skeleton, dtype=np.uint8)
        cv2.drawContours(mask, [cnt], -1, color=255, thickness=cv2.FILLED)

        # Filter skeleton with mask
        skeleton_masked = cv2.bitwise_and(skeleton, skeleton, mask=mask)
        skeleton_coords = np.column_stack(np.where(skeleton_masked == 255))

        if len(skeleton_coords) < 10:
            print("Too few skeleton points detected.")
            return

        # Sort using approximate path distance
        sorted_indices = np.argsort(skeleton_coords[:, 0] + skeleton_coords[:, 1])
        sorted_path = skeleton_coords[sorted_indices]

        # Interpolate and resample 50 evenly spaced points
        tck, u = splprep([sorted_path[:, 1], sorted_path[:, 0]], s=5)
        num_pts = 10
        u_fine = np.linspace(0, 1, num_pts)
        x_new, y_new = splev(u_fine, tck)
        path_pts = list(zip(x_new, y_new))
        # print(len(path_pts))

        # Get real world coordinates
        # Image coordinates (in pixels)
        image_pts = np.array([
            [0, 0],          # top-left
            [320, 0],        # top-right
            [0, 345],        # bottom-left
            [320, 345]       # bottom-right
        ], dtype=np.float32)

        # Real-world coordinates (in mm)
        world_pts = np.array([
            [-361.398, -194.96],     # top-left
            [-230.793, -188.862],   # top-right
            [-364.28, -326.304],    # bottom-left
            [-236.083, -334.496]    # bottom-right
        ], dtype=np.float32)

        # Compute homography matrix
        H, _ = cv2.findHomography(image_pts, world_pts)

        # path_pts_coor is [(x1, y1), (x2, y2), ...]
        img_pts = np.array(path_pts, dtype=np.float32).reshape(-1, 1, 2)
        world_coords = cv2.perspectiveTransform(img_pts, H)

        # Flatten
        world_coords = world_coords.reshape(-1, 2)

        return image_crop, path_pts, world_coords
    
    def show_target_pth(self, image_crop, path_pts, world_coords):
        # Mark correspondent points
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.2
        color = (0, 0, 255)  # red text
        thickness = 1

        # Draw image points and annotate with real-world coordinates
        for (img_pt, world_pt) in zip(path_pts, world_coords):
            x_img, y_img = int(img_pt[0]), int(img_pt[1])
            x_real, y_real = world_pt

            # Draw the image point
            cv2.circle(image_crop, (x_img, y_img), 3, color, -1)

            # Create text label with real-world coordinates (rounded for clarity)
            label = f"({x_real:.1f}, {y_real:.1f})"

            # Offset the text a little so it doesn't overlap the point
            text_pos = (x_img - 70, y_img + 5)

            # Put the text on the image
            cv2.putText(image_crop, label, text_pos, font, font_scale, color, thickness, cv2.LINE_AA)

        # cv2.imshow("Origin Image", org_image)
        cv2.imshow("Resampled Skeleton Path", image_crop)
        cv2.imwrite("path.jpg", image_crop)
        cv2.waitKey(5000)
        cv2.destroyAllWindows()

    def forward_kinematics(self, q):
        # th1, th2, th3, th4, th5, th6 = q
        th1, th2, th3, th6 = q
        th4 = -np.pi/2 - (th2 + th3)
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
        bounds = [
            (-np.pi, np.pi),     # q1
            (-np.pi/2, np.pi/2), # q2 
            (-np.pi, np.pi), # q3
            # (-np.pi/2, np.pi/2), # q4
            # (-np.pi/2, np.pi/2), # q5
            (-np.pi, np.pi)      # q6
        ]
        # bounds = [
        #     (-np.pi, np.pi),     # q1
        #     (-np.pi*3/2, np.pi/2), # q2 
        #     (-2.6179, 2.443), # q3
        #     # (-np.pi/2, np.pi/2), # q4
        #     # (-np.pi/2, np.pi/2), # q5
        #     (-np.pi, np.pi)      # q6
        # ]

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


    def publish_next_joint_state(self):
        if self.path_index >= len(self.path_world_coords):
            self.get_logger().info("All path points processed.")
            return
        else:
            point = self.path_world_coords[self.path_index]
            x, y = float(point[0]), float(point[1])


            self.desired_position = np.array([x, y, self.z])

            try:
                theta1, theta2, theta3, theta4, theta5, theta6 = self.inverse_kinematics(x, y, self.z)
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
                msg.position = [theta1, theta2, theta3, theta4, theta5, theta6]

                self.publisher.publish(msg)
                self.get_logger().info(f"IK: θ1={theta1:.2f}, θ2={theta2:.2f}, θ3={theta3:.2f}, θ4={theta4:.2f}, θ5={theta5:.2f}, θ6={theta6:.2f}")
                self.get_logger().info(f"Published joint angles for point {self.path_index}: ({x:.2f}, {y:.2f}, {self.z:.2f})")

                self.path_index += 1  # move to the next point

            except ValueError as e:
                self.get_logger().warn(f"IK failed for point {self.path_index}: {e}")
                self.path_index += 1  # still move on to the next point



    def reset_robot(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        msg.position = [0.0] * 6
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImgProcessPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
