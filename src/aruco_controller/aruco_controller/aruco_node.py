#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# Importy do sterowania robotem
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header

class ArucoNode(Node):
    def __init__(self):
        super().__init__('aruco_node')
        
        #Subskrypcja kamery
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)
        
        #Publikator komend do robota UR5
        self.publisher_ = self.create_publisher(
            JointTrajectory, 
            '/scaled_joint_trajectory_controller/joint_trajectory', 
            10)

        self.br = CvBridge()
        
        #Szerokosc strefy stopu
        self.dead_zone_pixels = 60 
        
        #Nazwy stawow robota UR5e (
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        
        #Zmienna zeby nie wysylac komend non-stop, tylko jak sie zmieni decyzja
        self.last_command = "NONE"

        self.get_logger().info('Startuje Aruco Node V5 (STEROWANIE ROBOTEM)!')

    def send_robot_command(self, position_type):
        # Jesli komenda jest taka sama jak poprzednia, ignorujemy
        if position_type == self.last_command:
            return

        msg = JointTrajectory()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        
        #DEFINICJA POZYCJI
        if position_type == "UP":
            #Unies ramie (drugi staw na -1.57 rad)
            point.positions = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
            self.get_logger().info('Wysylam komende: JAZDA W GORE')
            
        elif position_type == "DOWN":
            #Opusc ramie (drugi staw na -0.5 rad)
            point.positions = [0.0, -0.5, 0.0, -1.57, 0.0, 0.0]
            self.get_logger().info('Wysylam komende: JAZDA W DOL')
            
        else:
            return 
        #Czas na wykonanie ruchu
        point.time_from_start.sec = 5
        point.time_from_start.nanosec = 0
        
        msg.points = [point]
        self.publisher_.publish(msg)
        self.last_command = position_type

    def image_callback(self, data):
        current_frame = self.br.imgmsg_to_cv2(data, "bgr8")
        height, width, channels = current_frame.shape
        center_y = int(height / 2)
        
        blue_color = (255, 0, 0)

        top_threshold = center_y - self.dead_zone_pixels
        bottom_threshold = center_y + self.dead_zone_pixels
        
        #Rysowanie linii
        cv2.line(current_frame, (0, top_threshold), (width, top_threshold), blue_color, 2)
        cv2.line(current_frame, (0, bottom_threshold), (width, bottom_threshold), blue_color, 2)
        cv2.putText(current_frame, "STREFA STOP", (10, center_y + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, blue_color, 1)
        
        arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        arucoParams = cv2.aruco.DetectorParameters_create()
        (corners, ids, rejected) = cv2.aruco.detectMarkers(current_frame, arucoDict, parameters=arucoParams)

        if len(corners) > 0:
            ids = ids.flatten()
            for (markerCorner, markerID) in zip(corners, ids):
                corners_abcd = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners_abcd
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)

                cv2.aruco.drawDetectedMarkers(current_frame, corners)
                cv2.circle(current_frame, (cX, cY), 4, (0, 0, 255), -1)

                if cY < top_threshold:
                    cv2.putText(current_frame, "UP -> ROBOT GORA", (cX + 10, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    self.send_robot_command("UP")
                    
                elif cY > bottom_threshold:
                    cv2.putText(current_frame, "DOWN -> ROBOT DOL", (cX + 10, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    self.send_robot_command("DOWN")
                    
                else:
                    cv2.putText(current_frame, "STOP", (cX + 10, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.7, blue_color, 2)
                    self.last_command = "STOP" 
        else:
            cv2.putText(current_frame, "BRAK ZNACZNIKA", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        cv2.imshow("Panel Sterowania UR5", current_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    aruco_node = ArucoNode()
    rclpy.spin(aruco_node)
    aruco_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()