import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import numpy as np
import heapq
from scipy.optimize import minimize
import cv2
from scipy.interpolate import splprep, splev
import os
import matplotlib.pyplot as plt

class MazeSolverPublisher(Node):
    # def __init__(self):
    #     super().__init__('MazeSolver')
    #     self.publisher = self.create_publisher(JointState, '/joint_states', 1)
    #     self.timer = self.create_timer(0.01, self.publish_next_joint_state)

    #     ##########################################################################
    #     # Adjust these parameters
    #     self.home = np.array([
    #         [-383.111, -192.595],     # top-left
    #         [-222.111, -197.865],   # top-right
    #         [-369.510, -343.600],    # bottom-left
    #         [-227.516, -338.718]    # bottom-right
    #     ], dtype=np.float32)

    #     # Adjust these parameters based on cameras
    #     self.start_y, self.start_x, self.end_y, self.end_x = 40, 150, 385, 490
    #     self.start_y, self.start_x, self.end_y, self.end_x = 40, 120, 375, 470
    #     self.z = 160
    #     ##########################################################################

    #     self.path_index = 0
    #     self.path_pts, self.path_world_coords = self.get_path_pts()


    #     if self.path_world_coords is None:
    #         self.get_logger().error("No path points found.")
    #         rclpy.shutdown()
    #         return
    def __init__(self):
        super().__init__('MazeSolver')
        self.publisher = self.create_publisher(JointState, '/joint_states', 1)
        self.timer = self.create_timer(0.01, self.publish_next_joint_state)

        ##########################################################################
        # Adjust these parameters
        self.home = np.array([
            [-388.012, -179.438],
            [-234.425, -184.989],
            [-388.072, -330.395],
            [-241.391, -336.786]
        ], dtype=np.float32)

        self.start_y, self.start_x, self.end_y, self.end_x = 40, 120, 365, 460
        self.z = 160
        ##########################################################################

        self.path_index = 0
        self.path_pts, self.path_world_coords = self.get_path_pts()

        if self.path_world_coords is None:
            self.get_logger().error("No path points found.")
            rclpy.shutdown()
            return

        # ✅ Precompute IK for all points
        self.get_logger().info("Computing inverse kinematics for all path points...")
        self.ik_solutions = []
        for idx, point in enumerate(self.path_world_coords):
            x, y = float(point[0][0]), float(point[0][1])
            self.desired_position = np.array([x, y, self.z])
            ik = self.inverse_kinematics(x, y, self.z)
            if ik is not None:
                self.ik_solutions.append(ik)
            else:
                self.get_logger().warn(f"IK failed for point {idx}")
                self.ik_solutions.append([0.0] * 6)  # fallback

        self.get_logger().info("IK precomputation complete.")


    def get_path_pts(self):
        # Get sample images
        # img_save_pth = r'/home/andy/ASU/RAS545_Robotics/Lab/final_project/sample_imgs'
        i = '1'
        # img_pth = img_save_pth + '/sample_img' + i + '.jpg'
        # img_orig = cv2.imread(img_pth)

        # Get real life image
        img_orig = self.getImgIRL(self.start_y, self.start_x, self.end_y, self.end_x)
        img2 = img_orig.copy()

        cv2.imshow('Original Image', img_orig)

        # 1. Get the starting/ending points and the filtered regions
        centroids, filtered_regions = self.find_circle_centroid(img_orig)
        point1 = centroids[0]
        point2 = centroids[1]
        if self.color_detection(img2, point1) == 'red':
            starting_point = point1
            ending_point = point2
        else:
            starting_point = point2
            ending_point = point1
        xs, ys = starting_point
        xe, ye = ending_point
        start_color, end_color = [(255, 100, 255), (100, 255, 0)]
        cv2.putText(img2, 'start', (xs-20, ys+20), cv2.FONT_HERSHEY_COMPLEX, 0.5, start_color, 1)
        cv2.putText(img2, 'goal', (xe-20, ye+20), cv2.FONT_HERSHEY_COMPLEX, 0.5, end_color, 1)
        cv2.circle(img2, starting_point, 1, start_color, 5)
        cv2.circle(img2, ending_point, 1, end_color, 5)
        print(ending_point)
        

        # 2. Get the binary image and make the filtered regions white
        gray = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
        k = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        gray = cv2.morphologyEx(gray, cv2.MORPH_CLOSE, k)
        _, bw = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        # cv2.imshow('Binary image', bw)
        modified_bw = bw.copy()
        # Filter out the circles
        for fil_reg in filtered_regions:
            x, y, w, h = fil_reg
            modified_bw[x:x+w, y:y+h] = 0
        # cv2.imshow('Modified Binary image', modified_bw)
        wall_dilate_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (40, 40))  # Increase size for more margin
        dilated_bw = cv2.dilate(modified_bw, wall_dilate_kernel)
        # cv2.imshow('dilated image', dilated_bw)

        # 3. Use A* to find the path
        cost_map = self.make_cost_map(dilated_bw)
        path = self.astar_cost(cost_map, dilated_bw, starting_point, ending_point)
        # print(len(path))
        path_interval = 40
        path_pts = path[::path_interval]
        path_more_starting = [path_pts[0], path_pts[0]]*50
        # path_more_ending = [path_pts[-1], path_pts[-1]]*100
        # print(path_more_starting)
        path_pts_modified = path_more_starting + path_pts
        # print(path_pts_modified)
        path_pts_modified = path_pts_modified  + [ending_point]
        print(path_pts_modified)

        # 4. Draw Path on the original image
        sol_img = self.draw_path(img2, path_pts)
        cv2.imshow('Path Image', sol_img)
        save_img_name = r'/home/andy/ASU/RAS545_Robotics/Lab/final_project/report_imgs/path_img' + i + '.jpg'
        cv2.imwrite(save_img_name, sol_img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        # 5. Convert the path coordinates into real coordinates
        world_coords = []
        for path_pt in path_pts_modified:
            # print(path_pt)
            real_coor = self.getCoordsIRL(path_pt)       # real_coor.shape == (1, 1, 2)
            real_coor = real_coor.reshape(-1, 2)
            world_coords.append(real_coor)
        # print(len(world_coords))
        # print(len(path_pts))
    
        return path_pts, world_coords

    def getCoordsIRL(self, img_pt):
        # Image coordinates (in pixels)
        image_pts = np.array([
            [0, 0],                                             # top-left
            [self.end_x-self.start_x, 0],                       # top-right
            [0, self.end_y-self.start_y],                       # bottom-left
            [self.end_x-self.start_x, self.end_y-self.start_y]  # bottom-right
        ], dtype=np.float32)

        # Real-world coordinates (adjust every time base on the real coordinates) -- NEED To BE FIXED!!
        world_pts = self.home

        # Compute homography matrix
        H, _ = cv2.findHomography(image_pts, world_pts)
        img_pt = np.array([[img_pt]], dtype=np.float32)  # Reshape path_pt to (1, 1, 2)
        # Get the world coordinates with the point on image
        world_coords = cv2.perspectiveTransform(img_pt, H)     # Only one point cuz img_pts is one of the point on the path

        return world_coords
    
    def color_detection(self, img, point_coordinate):
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        center_x, center_y = point_coordinate
        pixel_hsv = hsv_img[center_y, center_x]
        hue, sat, val = pixel_hsv
        # print(f'H: {hue}, S: {sat}, V: {val}')

        color_name = None

        # Check for red-like color
        lower_red_hue1, upper_red_hue1 = 0, 20
        lower_red_hue2, upper_red_hue2 = 160, 180
        lower_sat, upper_sat = 10, 255
        lower_val, upper_val = 10, 255

        if ((lower_red_hue1 <= hue <= upper_red_hue1) or (lower_red_hue2 <= hue <= upper_red_hue2)) and \
        (lower_sat <= sat <= upper_sat) and (lower_val <= val <= upper_val):
            color_name = "red"

        # Check for green-like color
        lower_green_hue, upper_green_hue = 30, 80
        if (lower_green_hue <= hue <= upper_green_hue) and \
        (lower_sat <= sat <= upper_sat) and (lower_val <= val <= upper_val):
            color_name = "green"

        return color_name

    def find_circle_centroid(self, img):
        # edge detection
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 200) # Adjust thresholds as needed
        # cv2.imshow('edges', edges)
        # cv2.waitKey(0)

        # shape detection
        contours, hierarchy = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        centroids = []
        filtered_region = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area>100:
                length = cv2.arcLength(cnt, True)
                vertice = cv2.approxPolyDP(cnt, length*0.02, True)
                corners = len(vertice)
                # print(f'Area: {area}, corners: {corners}')
                if corners == 8:        # its a cricle
                    x, y, w, h = cv2.boundingRect(vertice) # mark shapes with rectangle
                    extend = 5
                    x, y, w, h = x-extend, y-extend, w+extend*2, h+extend*2
                    # cv2.rectangle(img_orig, (x,y), (x+w,y+h), (255,0,0), 2)
                    centroids.append((x+round(w/2), y+round(h/2)))
                    filtered_region.append((x, y, w, h))
        # print(centroids)
        return centroids, filtered_region


    def make_cost_map(self, free_mask):
        # free_mask: 0=free,255=wall
        dist = cv2.distanceTransform((free_mask==0).astype(np.uint8),
                                    cv2.DIST_L2,5)
        # invert so center=low cost
        maxd = dist.max()
        cost = (maxd - dist) + 1   # +1 so cost>=1
        return cost


    def astar_cost(self, cost, mask, start, goal):
        h, w = mask.shape
        def H(p,q): 
            return abs(p[0]-q[0]) + abs(p[1]-q[1])
        open_, gscore, prev = [(H(start,goal),0,start)], {start:0}, {}
        moves = [(1,0),(-1,0),(0,1),(0,-1)]
        while open_:
            _, g, (x,y) = heapq.heappop(open_)
            if (x,y)==goal: 
                break
            for dx,dy in moves:
                nx,ny = x+dx, y+dy
                if 0<=nx<w and 0<=ny<h and mask[ny,nx]==0:
                    ng = g + cost[ny,nx]
                    if ng < gscore.get((nx,ny),1e12):
                        gscore[(nx,ny)] = ng
                        prev[(nx,ny)] = (x,y)
                        heapq.heappush(open_, (ng + H((nx,ny),goal), ng, (nx,ny)))
        if goal not in prev:
            raise RuntimeError("No path")
        # Reconstruct
        path, cur = [], goal
        while cur!=start:
            path.append(cur)
            cur = prev[cur]
        path.append(start)
        return path[::-1]

    def draw_path(self, img, path):
        out = img.copy()
        for path_pt in path:
            cv2.circle(out, path_pt, 1, (255, 100, 0), 3)
        return out

    
    def getImgIRL(self, start_y, start_x, end_y, end_x):
        # Crop to region of interest
        # Capture the image
        cap = cv2.VideoCapture(0)
        ret, frame = cap.read()
        if ret:
            # cv2.imshow('hi', frame)
            frame = frame[start_y:end_y, start_x:end_x]
            # cv2.imshow('frame', frame)
            # cv2.waitKey(0)
        else:
            frame = None
            print("Cannot get the image!!!")
        return frame

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
            full_q = [th1, th2, th3, th4, th5, th6]
            full_q = [th1, th2, th3, th6]
            current_position = self.forward_kinematics(full_q)
            return np.sum((current_position - self.desired_position)**2)  # Squared error
        initial_guess = np.array([0.0, 0.0, 0.0, 0.0])
        # bounds = [
        #     (-np.pi, np.pi),     # q1
        #     (-np.pi/2, np.pi/2), # q2 
        #     (-np.pi, np.pi),     # q3
        #     (-np.pi/2, np.pi/2), # q4
        #     (-np.pi/2, np.pi/2), # q5
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
            # print(f"\nTrying method: {method}")
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


    # def publish_next_joint_state(self):
    #     if self.path_index >= len(self.path_world_coords):
    #         self.get_logger().info("All path points processed.")
    #         return
    #     else:
    #         point = self.path_world_coords[self.path_index]
    #         # print(point[0][1])
    #         x, y = float(point[0][0]), float(point[0][1])
    #         self.desired_position = np.array([x, y, self.z])
            
    #         try:
    #             theta1, theta2, theta3, theta4, theta5, theta6 = self.inverse_kinematics(x, y, self.z)
 
    #             msg = JointState()
    #             msg.header.stamp = self.get_clock().now().to_msg()
    #             msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    #             msg.position = [theta1, theta2, theta3, theta4, theta5, theta6]

    #             self.publisher.publish(msg)
    #             self.get_logger().info(f"IK: θ1={theta1:.2f}, θ2={theta2:.2f}, θ3={theta3:.2f}, θ4={theta4:.2f}, θ5={theta5:.2f}, θ6={theta6:.2f}")
    #             self.get_logger().info(f"Published joint angles for point {self.path_index}: ({x:.2f}, {y:.2f}, {self.z:.2f})")
    #             self.correct_index = self.path_index
    #             self.path_index += 1  # move to the next point
    #         except ValueError as e:
    #             self.get_logger().warn(f"IK failed for point {self.path_index}: {e}")
    #             self.path_index += 1  # still move on to the next point

    def publish_next_joint_state(self):
        if self.path_index >= len(self.path_world_coords):
            self.get_logger().info("All path points processed.")
            return

        current_point = self.path_world_coords[self.path_index]
    

        x, y = float(current_point[0][0]), float(current_point[0][1])
        self.desired_position = np.array([x, y, self.z])
        diffs = np.zeros(6)

        # current_angles = self.inverse_kinematics(x, y, self.z)
        current_angles = self.ik_solutions[self.path_index]
        if self.path_index == 0:
            self.correct_angles = current_angles
        
        for i in range(len(current_angles)):
            diffs[i] = abs(current_angles[i]-self.correct_angles[i])
        if max(diffs) < 0.2:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
            msg.position = current_angles

            self.publisher.publish(msg)
            # self.get_logger().info(f"Correct IK: θ1={self.correct_angles[0]:.2f}, θ2={self.correct_angles[1]:.2f}, θ3={self.correct_angles[2]:.2f}, θ4={self.correct_angles[3]:.2f}, θ5={self.correct_angles[4]:.2f}, θ6={self.correct_angles[5]:.2f}")
            self.get_logger().info(f"IK: θ1={current_angles[0]:.2f}, θ2={current_angles[1]:.2f}, θ3={current_angles[2]:.2f}, θ4={current_angles[3]:.2f}, θ5={current_angles[4]:.2f}, θ6={current_angles[5]:.2f}")
            # self.get_logger().info(f"Published index {self.path_index}: ({x:.2f}, {y:.2f}, {self.z:.2f})")
            # self.get_logger().info(f"Maximum difference: {max(diffs)}")
            # print()

            self.correct_angles = current_angles

        else:
            self.get_logger().warn(f"Skipped point {self.path_index} due to maximum Δθ = {max(diffs):.3f}")

        self.path_index += 1




def main(args=None):
    rclpy.init(args=args)
    node = MazeSolverPublisher()
    rclpy.spin(node)
    node.destroy_node()
if __name__ == '__main__':
    main()
