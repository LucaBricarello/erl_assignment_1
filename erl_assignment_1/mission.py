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
        self.latest_image = None  # Qui salviamo l'ultimo frame ricevuto
        
        self.marker_list = []     # L'array che deve arrivare a 5 elementi [cite: 12]
        self.detected_ids_set = set() # Set di appoggio per evitare duplicati rapidi
        
        self.mission_phase = "SEARCH" # Stati: "SEARCH", "VISIT", "DONE"
        self.current_target_index = 0 # Per scorrere l'array ordinato

        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.robot_pose = Point() # x, y
        self.robot_yaw = 0.0      # Orientamento (theta)

        # Ros timer 
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Nodo inizializzato. In attesa di immagini...")

    def image_callback(self, msg):
        """
        Callback eseguita ogni volta che arriva un frame dalla telecamera.
        Serve SOLO a convertire l'immagine e salvarla in self.latest_image.
        Non mettere logica complessa qui per evitare blocchi.
        """
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Errore conversione immagine: {e}')

    def odom_callback(self, msg):
        self.robot_pose.x = msg.pose.pose.position.x
        self.robot_pose.y = msg.pose.pose.position.y
        
        # Conversione da Quaternione a Eulero (Yaw)
        q = msg.pose.pose.orientation
        q_list = [q.x, q.y, q.z, q.w]
        (roll, pitch, yaw) = euler_from_quaternion(q_list)
        self.robot_yaw = yaw

    def control_loop(self):
        """
        IL CUORE DEL NODO.
        Qui implementi la logica:
        - Se array < 5 -> Ruota e cerca
        - Se array == 5 -> Esegui movimento verso il target
        """
        if self.latest_image is None:
            return # Aspettiamo la prima immagine

        # First state: rotate and search markers
        if self.mission_phase == "SEARCH":
            self.phase_search_markers()
        
        # Second state: visit markers in order
        elif self.mission_phase == "VISIT":
            self.phase_visit_markers()

        # Third state: return to origin
        elif self.mission_phase == "TO_ORIGIN":
            self.phase_back_to_origin()

        # Third state: return to origin
        elif self.mission_phase == "DONE":
            self.get_logger().info("Missione completata! Tutti i marker visitati.", throttle_duration_sec=5)
            # terminate the ros node
            rclpy.shutdown()
            
        # Stop state
        else:
            self.stop_robot()

    # --- METODI PER I COMPORTAMENTI (DA COMPLETARE) ---

    def phase_search_markers(self):
        """
        Logica per ruotare il robot e cercare marker.
        """
        self.get_logger().info(f"Fase SEARCH: Trovati {len(self.marker_list)}/5", throttle_duration_sec=2)

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
            twist.angular.z = 0.6
            self.publisher_vel.publish(twist)
        else:
            self.stop_robot()

            # sorting the list in order
            self.marker_list.sort()
            self.get_logger().info("5 markers found! Proceeding to VISIT phase. the list is " + str(self.marker_list), throttle_duration_sec=2)

            self.mission_phase = "VISIT"

    def phase_visit_markers(self):
        """
        Logica per raggiungere i marker in ordine.
        """

        if self.current_target_index >= len(self.marker_list):
            self.mission_phase = "DONE"
            return

        target_id = self.marker_list[self.current_target_index]

        # Assicurati di avere un'immagine
        if self.latest_image is None:
            return
        
        # 1. Cerca target_id nell'immagine corrente
        # --- RILEVAMENTO MARKER ---
        # Necessario definire dizionario e parametri (meglio se in __init__, ma li metto qui per chiarezza)
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        parameters = cv2.aruco.DetectorParameters_create()
        
        # detectMarkers ritorna i corners [cite: 22]
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
            self.get_logger().info(f"Cercando ID {target_id}...")

            msg = Twist()

            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.6

            self.publisher_vel.publish(msg)
        
        else:
            self.get_logger().info(f"ID {target_id} found in image.")

            # target_corners[i][j] where i is the choosen corner and j is the choosen axis
            marker_center_x = (target_corners[0][0] + target_corners[1][0] + target_corners[2][0] + target_corners[3][0]) / 4
            marker_center_y = (target_corners[0][1] + target_corners[1][1] + target_corners[2][1] + target_corners[3][1]) / 4

            # Parametri immagine
            image_center_x = self.latest_image.shape[1] / 2

            error_x = image_center_x - marker_center_x

            # computing marker_perceived_width as difference between corner 0 (high left corner) and corner 1 (high right corner) then doing norm
            marker_perceived_width = np.linalg.norm(target_corners[0] - target_corners[1])
            self.get_logger().info(f"marker_perceived_width: {marker_perceived_width}.")

            is_alligned = False
            is_close = False

            if abs(error_x) < 0.03:
                is_alligned = True

            # marker_perceived_width is in pixels, so we set a threshold in pixels too
            if abs(marker_perceived_width) > 60:
                is_close = True

            k_p = 0.012

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

                msg.linear.x = 0.5
                msg.linear.y = 0.0
                msg.linear.z = 0.0
                msg.angular.x = 0.0
                msg.angular.y = 0.0
                msg.angular.z = k_p * error_x

                self.publisher_vel.publish(msg)

                return

            else:

                self.stop_robot()

                # Disegna cerchio sul marker
                marker_center = (int(marker_center_x), int(marker_center_y))
                cv2.circle(self.latest_image, marker_center, 50, (0, 255, 0), 3)

                # Pubblica immagine processata
                try:
                    proc_img_msg = self.bridge.cv2_to_imgmsg(self.latest_image, "bgr8")
                    self.publisher_proc_img.publish(proc_img_msg)
                except Exception as e:
                    self.get_logger().error(f'Errore pubblicazione immagine processata: {e}')

                self.mission_phase = "TO_ORIGIN"

                return

    def phase_back_to_origin(self):
        """
        Riporta il robot a 0,0 usando l'odometria.
        """
        # 1. Calcola errore di distanza
        dist_error = math.sqrt(self.robot_pose.x**2 + self.robot_pose.y**2)
        
        # 2. Calcola angolo desiderato verso l'origine (atan2(dy, dx))
        # dy = 0 - y, dx = 0 - x
        desired_yaw = math.atan2(-self.robot_pose.y, -self.robot_pose.x)
        
        # Calcola errore angolare normalizzato tra -pi e pi
        yaw_error = desired_yaw - self.robot_yaw
        # Normalizzazione angolo (importante per non fare giri di 360 inutili)
        while yaw_error > math.pi: yaw_error -= 2 * math.pi
        while yaw_error < -math.pi: yaw_error += 2 * math.pi

        msg = Twist()
        
        # SOGLIE
        DIST_TOLERANCE = 0.1  # metri (es. 10 cm)
        YAW_TOLERANCE = 0.05  # radianti (~3 gradi)

        if dist_error < DIST_TOLERANCE:
            # Siamo arrivati all'origine!
            self.stop_robot()
            self.get_logger().info("Tornato all'origine. Passo al prossimo marker.")
            
            # LOGICA DI TRANSIZIONE
            self.current_target_index += 1
            self.mission_phase = "VISIT" # O qualsiasi nome usi per tornare a cercare il prossimo
            return

        # LOGICA DI MOVIMENTO (Gira poi vai dritto)
        if abs(yaw_error) > YAW_TOLERANCE:
            # Se non stiamo guardando l'origine, ruotiamo sul posto
            msg.linear.x = 0.0
            msg.angular.z = 0.7 * yaw_error # Controllo P sull'angolo
        else:
            # Se stiamo guardando l'origine, andiamo dritti
            msg.linear.x = 0.7 # Velocità costante o proporzionale alla distanza
            msg.angular.z = 0.0 # (Opzionale: puoi lasciare un piccolo correttivo angolare)
        
        self.publisher_vel.publish(msg)

    def stop_robot(self):
        """Funzione helper per fermare il robot immediatamente"""
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