#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage # MODIFICATION 1: Import necessary
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from tf_transformations import euler_from_quaternion
import math

class AssignmentNode(Node):
    def __init__(self):
        super().__init__('assignment_node')

        self.camera_topic = '/camera/rgb/image_raw/compressed'
        self.cmd_vel_topic = '/cmd_vel'
        self.output_image_topic = '/robot/processed_image'

        # Image subscriber
        # MODIFICATION 2: The message type must be CompressedImage
        self.subscription = self.create_subscription(
            CompressedImage,
            self.camera_topic,
            self.image_callback,
            10)
        
        # Command velocity publisher
        self.publisher_vel = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        
        # Image publisher for processed images
        # Note: output remains 'Image' raw, a meno che tu non voglia pubblicare compresso anche quello
        self.publisher_proc_img = self.create_publisher(Image, self.output_image_topic, 10)

        self.bridge = CvBridge()
        self.latest_image = None 
        
        self.marker_list = []     
        self.detected_ids_set = set() 
        
        self.mission_phase = "SEARCH" 
        self.current_target_index = 0 

        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.robot_pose = Point() 
        self.robot_yaw = 0.0      

        # Ros timer 
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Node initialized. Waiting for compressed images...")

    def image_callback(self, msg):
        """
        Callback to handle COMPRESSED images.
        """
        try:
            # MODIFICATION 3: Conversion from CompressedImage to OpenCV
            # Using CvBridge method for compressed images
            self.latest_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def odom_callback(self, msg):
        self.robot_pose.x = msg.pose.pose.position.x
        self.robot_pose.y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        q_list = [q.x, q.y, q.z, q.w]
        (roll, pitch, yaw) = euler_from_quaternion(q_list)
        self.robot_yaw = yaw

    def control_loop(self):
        if self.latest_image is None:
            return 

        if self.mission_phase == "SEARCH":
            self.phase_search_markers()
        
        elif self.mission_phase == "VISIT":
            self.phase_visit_markers()

        elif self.mission_phase == "TO_ORIGIN":
            self.phase_back_to_origin()

        elif self.mission_phase == "DONE":
            self.get_logger().info("Mission completed! All markers visited.", throttle_duration_sec=5)
            self.stop_robot()
            
        else:
            self.stop_robot()

    def phase_search_markers(self):
        # self.get_logger().info(f"SEARCH phase: Found {len(self.marker_list)}/5", throttle_duration_sec=2)

        img_gray = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2GRAY)

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        parameters = cv2.aruco.DetectorParameters_create()

        corners, ids, _ = cv2.aruco.detectMarkers(img_gray, aruco_dict, parameters=parameters)

        if ids is not None:
            ids_list = ids.flatten().tolist()
            for marker_id in ids_list:
                if int(marker_id) not in self.detected_ids_set:
                    self.detected_ids_set.add(int(marker_id))
                    self.marker_list.append(int(marker_id))
                    self.get_logger().info(f"New marker found: {marker_id}  ({len(self.marker_list)}/5)")

        if len(self.marker_list) < 5:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.3
            self.publisher_vel.publish(twist)
        else:
            self.stop_robot()
            self.marker_list.sort()
            self.get_logger().info("5 markers found! Proceeding to VISIT phase: " + str(self.marker_list))
            self.mission_phase = "VISIT"

    def phase_visit_markers(self):
        if self.current_target_index >= len(self.marker_list):
            self.mission_phase = "DONE"
            return

        target_id = self.marker_list[self.current_target_index]

        if self.latest_image is None:
            return
        
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        parameters = cv2.aruco.DetectorParameters_create()
        
        corners, ids, _ = cv2.aruco.detectMarkers(self.latest_image, aruco_dict, parameters=parameters)

        found_target = False
        target_corners = None

        if ids is not None:
            for i, id in enumerate(ids):
                if id == target_id:
                    found_target = True
                    target_corners = corners[i][0]
                    break
                
        if not found_target:
            self.get_logger().info(f"Searching ID {target_id}...", throttle_duration_sec=2)
            msg = Twist()
            msg.angular.z = 0.25
            self.publisher_vel.publish(msg)
        
        else:
            # self.get_logger().info(f"ID {target_id} found.")
            marker_center_x = (target_corners[0][0] + target_corners[1][0] + target_corners[2][0] + target_corners[3][0]) / 4
            # marker_center_y = ... (not used in current control)

            image_center_x = self.latest_image.shape[1] / 2
            error_x = image_center_x - marker_center_x
            
            # Calculate apparent marker width
            marker_perceived_width = np.linalg.norm(target_corners[0] - target_corners[1])
            
            is_aligned = abs(error_x) < 3
            is_close = abs(marker_perceived_width) > 90 # Slightly increased for safety
            k_p = 0.0005 # Slightly reduced gain for stability

            msg = Twist()

            if not is_aligned:
                msg.angular.z = k_p * error_x
                self.publisher_vel.publish(msg)
                return

            elif is_aligned and not is_close:
                msg.linear.x = 0.2 # Approach speed
                msg.angular.z = k_p * error_x
                self.publisher_vel.publish(msg)
                return

            else:
                self.stop_robot()
                self.get_logger().info(f"Arrived at target {target_id}. Returning to base.")

                # Draw circle and publish processed image
                marker_center = (int(marker_center_x), int((target_corners[0][1] + target_corners[2][1])/2))
                cv2.circle(self.latest_image, marker_center, 65, (0, 255, 0), 3)

                try:
                    # Note: Publishing a RAW image here, so cv2_to_imgmsg is appropriate
                    proc_img_msg = self.bridge.cv2_to_imgmsg(self.latest_image, "bgr8")
                    self.publisher_proc_img.publish(proc_img_msg)
                except Exception as e:
                    self.get_logger().error(f'Error publishing image: {e}')
                self.mission_phase = "TO_ORIGIN"
                return

    def phase_back_to_origin(self):
        dist_error = math.sqrt(self.robot_pose.x**2 + self.robot_pose.y**2)
        desired_yaw = math.atan2(-self.robot_pose.y, -self.robot_pose.x)
        
        yaw_error = desired_yaw - self.robot_yaw
        while yaw_error > math.pi: yaw_error -= 2 * math.pi
        while yaw_error < -math.pi: yaw_error += 2 * math.pi

        msg = Twist()
        
        DIST_TOLERANCE = 0.05 
        YAW_TOLERANCE = 0.05 

        if dist_error < DIST_TOLERANCE:
            self.stop_robot()
            self.get_logger().info("Returned to origin. Moving to next target.")
            self.current_target_index += 1
            self.mission_phase = "VISIT"
            return

        if abs(yaw_error) > YAW_TOLERANCE:
            msg.angular.z = 0.7 * yaw_error
        else:
            msg.linear.x = 0.5
            msg.angular.z = 0.1 * yaw_error # Small correction while moving forward
        
        self.publisher_vel.publish(msg)

    def stop_robot(self):
        msg = Twist()
        self.publisher_vel.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AssignmentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()