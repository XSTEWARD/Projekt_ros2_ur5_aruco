#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ArucoNode(Node):
    def __init__(self):
        super().__init__('aruco_node')
        
        # Subskrypcja kamery 
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)
        
        self.br = CvBridge()
        self.get_logger().info('Startuje Aruco Node! Pokaz znacznik na kamerze.')

    def image_callback(self, data):
        #Konwersja ROS -> OpenCV
        current_frame = self.br.imgmsg_to_cv2(data, "bgr8")
        
        #Konfiguracja ArUco (uzywamy slownika 4x4)
        arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        arucoParams = cv2.aruco.DetectorParameters()
        
        #Detekcja
        (corners, ids, rejected) = cv2.aruco.detectMarkers(current_frame, arucoDict, parameters=arucoParams)

        height, width, channels = current_frame.shape
        center_y_screen = height / 2

        #Wykryto znacznik
        if len(corners) > 0:
            
            ids = ids.flatten()
            
            for (markerCorner, markerID) in zip(corners, ids):
                # Obliczanie srodka znacznika
                corners_abcd = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners_abcd
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)

                # Rysowanie ramki i srodka
                cv2.aruco.drawDetectedMarkers(current_frame, corners)
                cv2.circle(current_frame, (cX, cY), 4, (0, 0, 255), -1)

                #LOGIKA STEROWANIA
                if cY < center_y_screen:
                    cv2.putText(current_frame, "UP (Jazda w gore)", (cX, cY - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    self.get_logger().info(f'Marker ID {markerID}: GÓRA (Y={cY} < {center_y_screen})')
                    # TUTAJ WSTAWIMY PUBLIKACJE DO ROBOTA UR5
                else:
                    cv2.putText(current_frame, "DOWN (Jazda w dol)", (cX, cY + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    self.get_logger().info(f'Marker ID {markerID}: DÓŁ (Y={cY} > {center_y_screen})')
                    # TUTAJ  WSTAWIMY PUBLIKACJE DO ROBOTA UR5

        #Wyswietlanie okna
        cv2.imshow("Kamera ArUco", current_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    aruco_node = ArucoNode()
    rclpy.spin(aruco_node)
    aruco_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()