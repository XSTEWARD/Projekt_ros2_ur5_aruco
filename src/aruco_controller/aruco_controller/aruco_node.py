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
        # Parametr okreslajacy jak szeroka jest strefa srodkowa (w pikselach)
        self.dead_zone_pixels = 60 
        self.get_logger().info('Startuje Aruco Node V4 (Clean UI)!')

    def image_callback(self, data):
        current_frame = self.br.imgmsg_to_cv2(data, "bgr8")
        
        #Konfiguracja czesci wizualnej
        height, width, channels = current_frame.shape
        center_y = int(height / 2)
        
        blue_color = (255, 0, 0)

        # Rysowanie strefy STOP (Niebieskie linie)
        top_threshold = center_y - self.dead_zone_pixels
        bottom_threshold = center_y + self.dead_zone_pixels
        
        cv2.line(current_frame, (0, top_threshold), (width, top_threshold), blue_color, 2)
        cv2.line(current_frame, (0, bottom_threshold), (width, bottom_threshold), blue_color, 2)
        
        #Detekcja Aruco
        arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        arucoParams = cv2.aruco.DetectorParameters_create()
        (corners, ids, rejected) = cv2.aruco.detectMarkers(current_frame, arucoDict, parameters=arucoParams)

        #Logika
        if len(corners) > 0:
            ids = ids.flatten()
            for (markerCorner, markerID) in zip(corners, ids):
                corners_abcd = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners_abcd
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)

                cv2.aruco.drawDetectedMarkers(current_frame, corners)
                cv2.circle(current_frame, (cX, cY), 4, (0, 0, 255), -1)

                #logika ze strefa STOP
                if cY < top_threshold:
                    #Znacznik wysoko
                    cv2.putText(current_frame, "UP (Jazda w gore)", (cX + 10, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    # self.get_logger().info('KOMENDA: GÓRA')
                elif cY > bottom_threshold:
                    #Znacznik nisko
                    cv2.putText(current_frame, "DOWN (Jazda w dol)", (cX + 10, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    # self.get_logger().info('KOMENDA: DÓŁ')
                else:
                    #Znacznik w srodku
                    cv2.putText(current_frame, "STOP (Srodek)", (cX + 10, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.7, blue_color, 2)
                    # self.get_logger().info('KOMENDA: STOP')
        else:
            # Gdy nie widzi znacznika
            cv2.putText(current_frame, "BRAK ZNACZNIKA", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        cv2.imshow("Kamera ArUco V4", current_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    aruco_node = ArucoNode()
    rclpy.spin(aruco_node)
    aruco_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()