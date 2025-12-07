#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
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

        # Topics
        self.camera_topic = '/camera/image'
        self.cmd_vel_topic = '/cmd_vel'
        self.output_image_topic = '/robot/processed_image'

        # Image subscriber
        self.subscription = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10)
        
        # Command velocity publisher
        self.publisher_vel = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        
        # Image publisher for processed images
        self.publisher_proc_img = self.create_publisher(Image, self.output_image_topic, 10)

        self.bridge = CvBridge()
        self.latest_image = None  # Here we will store the latest image frame
        
        self.marker_list = []     # The array that must reach 5 elements
        self.detected_ids_set = set() # Support set to avoid rapid duplicates
        
        self.mission_phase = "SEARCH" # States: "SEARCH", "VISIT", "DONE", "TO_ORIGIN"
        self.current_target_index = 0 # To iterate through the sorted array

        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.robot_pose = Point() # x, y
        self.robot_yaw = 0.0      # Orientation (theta)

        # Ros timer 
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Node initialized. Waiting for images...")

    def image_callback(self, msg):
        """
        Callback executed every time a frame arrives from the camera.
        It ONLY converts the image and saves it in self.latest_image.
        """
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def odom_callback(self, msg):
        self.robot_pose.x = msg.pose.pose.position.x
        self.robot_pose.y = msg.pose.pose.position.y
        
        # Conversion from Quaternion to Euler (Yaw)
        q = msg.pose.pose.orientation
        q_list = [q.x, q.y, q.z, q.w]
        (roll, pitch, yaw) = euler_from_quaternion(q_list)
        self.robot_yaw = yaw

    def control_loop(self):
        """
        MAIN CONTROL LOOP
        """
        if self.latest_image is None:
            return # Waiting for the first image

        # First state: rotate and search markers
        if self.mission_phase == "SEARCH":
            self.phase_search_markers()
        
        # Second state: visit markers in order
        elif self.mission_phase == "VISIT":
            self.phase_visit_markers()

        # Third state: return to origin
        elif self.mission_phase == "TO_ORIGIN":
            self.phase_back_to_origin()

        # Last state: Mission completed
        elif self.mission_phase == "DONE":
            self.get_logger().info("Mission completed! All markers visited.", throttle_duration_sec=5)
            # wait a bit before closing any windows
            cv2.waitKey(3000)
            # close any OpenCV windows
            cv2.destroyAllWindows()
            
        # Stop state
        else:
            self.stop_robot()

    def phase_search_markers(self):
        """
        Logic to rotate the robot and search for markers.
        """
        self.get_logger().info(f"SEARCH phase: Found {len(self.marker_list)}/5", throttle_duration_sec=2)

        # Convert latest image to grayscale
        img_gray = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2GRAY)

        # Prepare aruco dictionary & detector parameters
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        parameters = cv2.aruco.DetectorParameters_create()

        # Detect markers
        corners, ids, _ = cv2.aruco.detectMarkers(img_gray, aruco_dict, parameters=parameters)

        # If we detected something, add new ids to the list/set
        if ids is not None:
            # ids usually comes as Nx1 array -> flatten to python list
            ids_list = ids.flatten().tolist()
            for marker_id in ids_list:
                if int(marker_id) not in self.detected_ids_set:
                    self.detected_ids_set.add(int(marker_id))
                    self.marker_list.append(int(marker_id))
                    self.get_logger().info(f"New marker found: {marker_id}  ({len(self.marker_list)}/5)", throttle_duration_sec=2)

        # If we still need markers -> rotate in place to look around
        if len(self.marker_list) < 5:
            twist = Twist()
            twist.linear.x = 0.0
            # tune this angular speed if needed
            twist.angular.z = 0.4
            self.publisher_vel.publish(twist)
        else:
            self.stop_robot()

            # sorting the list in order
            self.marker_list.sort()
            self.get_logger().info("5 markers found! Proceeding to VISIT phase. the list is " + str(self.marker_list), throttle_duration_sec=2)

            self.mission_phase = "VISIT"

    def phase_visit_markers(self):
        """
        Logic to reach the markers in order.
        """

        if self.current_target_index >= len(self.marker_list):
            self.mission_phase = "DONE"
            return

        target_id = self.marker_list[self.current_target_index]

        # Make sure you have an image
        if self.latest_image is None:
            return
        
        # Search for target_id in the current image
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        parameters = cv2.aruco.DetectorParameters_create()
        
        # detectMarkers returns the corners
        corners, ids, _ = cv2.aruco.detectMarkers(self.latest_image, aruco_dict, parameters=parameters)

        found_target = False
        target_corners = None

        if ids is not None:
            for i, id in enumerate(ids):
                if id == target_id:
                    found_target = True
                    target_corners = corners[i][0]
                    break
                
        if found_target == False:
            self.get_logger().info(f"Searching for ID {target_id}...")

            msg = Twist()

            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.4

            self.publisher_vel.publish(msg)
        
        else:
            self.get_logger().info(f"ID {target_id} found in image.")

            # target_corners[i][j] where i is the choosen corner and j is the choosen axis
            marker_center_x = (target_corners[0][0] + target_corners[1][0] + target_corners[2][0] + target_corners[3][0]) / 4
            marker_center_y = (target_corners[0][1] + target_corners[1][1] + target_corners[2][1] + target_corners[3][1]) / 4

            # Image parameters
            image_center_x = self.latest_image.shape[1] / 2

            error_x = image_center_x - marker_center_x
            self.get_logger().info(f"error_x: {error_x}.")

            # computing marker_perceived_width as difference between corner 0 (high left corner) and corner 1 (high right corner) then doing norm
            marker_perceived_width = np.linalg.norm(target_corners[0] - target_corners[1])
            self.get_logger().info(f"marker_perceived_width: {marker_perceived_width}.")

            is_alligned = False
            is_close = False

            # error_x is computed as an average in pixels so it will have values multiples of 0.25
            if abs(error_x) < 3:
                is_alligned = True

            # marker_perceived_width is in pixels, so we set a threshold in pixels too
            if abs(marker_perceived_width) > 50:
                is_close = True

            k_p = 0.01

            msg = Twist()

            if is_alligned == False:

                msg.linear.x = 0.0
                msg.linear.y = 0.0
                msg.linear.z = 0.0
                msg.angular.x = 0.0
                msg.angular.y = 0.0
                msg.angular.z = k_p * error_x

                self.publisher_vel.publish(msg)

                return

            elif is_alligned == True and is_close == False:

                msg.linear.x = 0.4
                msg.linear.y = 0.0
                msg.linear.z = 0.0
                msg.angular.x = 0.0
                msg.angular.y = 0.0
                msg.angular.z = k_p * error_x

                self.publisher_vel.publish(msg)

                return

            else:

                self.stop_robot()

                # draw circle around the marker in the image
                marker_center = (int(marker_center_x), int(marker_center_y))
                cv2.circle(self.latest_image, marker_center, 45, (0, 255, 0), 3)

                # Publish processed image
                try:
                    proc_img_msg = self.bridge.cv2_to_imgmsg(self.latest_image, "bgr8")
                    self.publisher_proc_img.publish(proc_img_msg)
                except Exception as e:
                    self.get_logger().error(f'Error publishing processed image: {e}')

                cv2.namedWindow("Marker Found", cv2.WINDOW_NORMAL)
                cv2.resizeWindow("Marker Found", 600, 600)
                self.get_logger().info("Opening window to show marker found...", throttle_duration_sec=5)
                cv2.imshow("Marker Found", self.latest_image)
                cv2.waitKey(3000)

                self.mission_phase = "TO_ORIGIN"

                return

    def phase_back_to_origin(self):
        """
        Bring the robot back to 0,0 using odometry.
        """
        # Calculate distance error to origin
        dist_error = math.sqrt(self.robot_pose.x**2 + self.robot_pose.y**2)
        
        # Calculate desired angle towards origin (atan2(dy, dx))
        # dy = 0 - y, dx = 0 - x
        desired_yaw = math.atan2(-self.robot_pose.y, -self.robot_pose.x)
        
        # Calculate normalized angular error between -pi and pi
        yaw_error = desired_yaw - self.robot_yaw
        # Angle normalization (important to avoid unnecessary 360-degree rotations)
        while yaw_error > math.pi: yaw_error -= 2 * math.pi
        while yaw_error < -math.pi: yaw_error += 2 * math.pi

        msg = Twist()
        
        # THRESHOLDS
        DIST_TOLERANCE = 0.04  # meters (e.g., 4 cm)
        YAW_TOLERANCE = 0.05  # radians (~3 degrees)

        if dist_error < DIST_TOLERANCE:
            # We have arrived at the origin
            self.stop_robot()
            self.get_logger().info("Returned to origin. Moving to next marker.")
            
            # TRANSITION LOGIC
            self.current_target_index += 1
            self.mission_phase = "VISIT"
            return

        # MOVEMENT LOGIC
        if abs(yaw_error) > YAW_TOLERANCE:
            # If we are not facing the origin, rotate in place
            msg.linear.x = 0.0
            msg.angular.z = 0.7 * yaw_error # P control on angle
        else:
            # If we are facing the origin, go straight
            msg.linear.x = 0.7 
            msg.angular.z = 0.0 
        
        self.publisher_vel.publish(msg)

    def stop_robot(self):
        """Helper function to stop the robot immediately"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
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