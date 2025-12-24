#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge
import cv2
import numpy as np

#Importy do sterowania robotem
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
        
        #Subskrypcja pozycji STOP
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
            
        #Publikator komend
        self.publisher_ = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10)

        self.br = CvBridge()
        self.dead_zone_pixels = 60 
        
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        
        self.current_joints = []
        self.last_command = "NONE"
        
        self.get_logger().info('Startuje Aruco Node V10 (KLASYK + PAMIEC)!')

    def joint_state_callback(self, msg):
        if len(self.current_joints) == 0:
            self.current_joints = [0.0] * 6
        try:
            name_to_pos = {name: pos for name, pos in zip(msg.name, msg.position)}
            self.current_joints = [name_to_pos[j] for j in self.joint_names]
        except KeyError:
            pass

    def stop_robot(self):
        #STOP - zatrzymuje w aktualnej pozycji
        if not self.current_joints:
            return

        msg = JointTrajectory()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = self.current_joints # Zostan tu gdzie jestes
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 500000000 # 0.5s stop
        
        msg.points = [point]
        self.publisher_.publish(msg)

    def send_robot_command(self, position_type):
        #Jesli komenda ta sama co ostatnio, to ignoruj chyba ze STOP
        if position_type == self.last_command and position_type != "STOP":
            return

        msg = JointTrajectory()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()

        if position_type == "UP":
            #Pozycja: PIONOWO
            #Drugi staw: -1.57 
            point.positions = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
            self.get_logger().info('>>> JAZDA W GORE')
            
        elif position_type == "DOWN":
            #Pozycja: POZIOMO 
            #Drugi staw: -0.5
            point.positions = [0.0, -0.5, 0.0, -1.57, 0.0, 0.0]
            self.get_logger().info('>>> JAZDA W DOL')
            
        elif position_type == "STOP":
            self.stop_robot()
            self.last_command = "STOP"
            return

        point.time_from_start.sec = 5 # Czas na dojazd
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
        
        cv2.line(current_frame, (0, top_threshold), (width, top_threshold), blue_color, 2)
        cv2.line(current_frame, (0, bottom_threshold), (width, bottom_threshold), blue_color, 2)
        
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
                    cv2.putText(current_frame, "UP", (cX + 10, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    self.send_robot_command("UP")
                    
                elif cY > bottom_threshold:
                    cv2.putText(current_frame, "DOWN", (cX + 10, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    self.send_robot_command("DOWN")
                    
                else:
                    cv2.putText(current_frame, "STOP", (cX + 10, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.7, blue_color, 2)
                    self.send_robot_command("STOP")
        else:
            #Pamieta ostatni ruch
            cv2.putText(current_frame, "BRAK ZNACZNIKA - PAMIETAM RUCH", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

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