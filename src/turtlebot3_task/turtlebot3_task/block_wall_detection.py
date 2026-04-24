#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge # Translates ROS images to OpenCV
import cv2
import cv2.aruco as aruco
import numpy as np
from geometry_msgs.msg import Point

class GazeboVisionNode(Node):
    def __init__(self):
        super().__init__('gazebo_vision_node')
        
        # 1. ROS2 Setup
        # Gazebo TurtleBot3 camera topic is usually /camera/image_raw
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw', 
            self.image_callback,
            10)
        self.bridge = CvBridge()

        self.block_pub = self.create_publisher(Point, '/block_info', 10)

        # 2. ArUco Setup (Your version-compatible logic)
        try:
            self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
            self.parameters = aruco.DetectorParameters()
            self.detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)
            print("Using Modern ArUco Detector")
        except AttributeError:
            self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
            self.parameters = aruco.DetectorParameters_create()
            self.detector = None
            print("Using Legacy ArUco Detector")

    def image_callback(self, msg):
        # Convert the Gazebo image to an OpenCV frame
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # --- YOUR EXACT VALUES AND LOGIC ---
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Blue Mask
        lower_blue = np.array([110, 150, 50]) 
        upper_blue = np.array([125, 255, 255])
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

        # # Red Mask
        # lower_red = np.array([165, 150, 70]) 
        # upper_red = np.array([180, 255, 255])
        # mask_red = cv2.inRange(hsv, lower_red, upper_red)

        # Red Mask - Range 1 (Wrap-around start)
        lower_red_low = np.array([0, 150, 50])
        upper_red_low = np.array([10, 255, 255])
        mask_red_low = cv2.inRange(hsv, lower_red_low, upper_red_low)

        # Red Mask - Range 2 (Wrap-around end)
        lower_red_high = np.array([160, 150, 50]) # Lowered from 165 to be more inclusive
        upper_red_high = np.array([180, 255, 255])
        mask_red_high = cv2.inRange(hsv, lower_red_high, upper_red_high)

        # Combine both masks
        mask_red = cv2.addWeighted(mask_red_low, 1.0, mask_red_high, 1.0, 0.0)

        # --- ARUCO DETECTION ---
        if self.detector:
            corners, ids, rejected = self.detector.detectMarkers(frame)
        else:
            corners, ids, rejected = aruco.detectMarkers(frame, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
            for i, marker_id in enumerate(ids):
                cv2.putText(frame, f"WALL ID: {marker_id[0]}", (10, 50 + i*35), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # --- TARGETING: DRAW BOXES ---
        # Blue
        blue_cnts, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in blue_cnts:
            if cv2.contourArea(cnt) > 500: # Lowered for Gazebo distance
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)

                msg = Point()
                msg.x = float(x + w//2) # Horizontal center
                msg.y = float(cv2.contourArea(cnt)) # Using area as a "distance" proxy
                msg.z = 2.0 # 2 for blue
                self.block_pub.publish(msg)

                # PRINT COORDINATES TO TERMINAL
                print(f"BLUE CUBE detected at X: {x + w//2}")

        # Red
        red_cnts, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in red_cnts:
            if cv2.contourArea(cnt) > 500:
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)

                msg = Point()
                msg.x = float(x + w//2) # Horizontal center
                msg.y = float(cv2.contourArea(cnt)) # Using area as a "distance" proxy
                msg.z = 1.0 # 1 for blue
                self.block_pub.publish(msg)

                # PRINT COORDINATES TO TERMINAL
                print(f"RED CUBE detected at X: {x + w//2}")

        # --- DISPLAY ---
        cv2.imshow('Gazebo Robot View', frame)
        cv2.imshow('Red Mask', mask_red)
        cv2.imshow('Blue Mask', mask_blue)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = GazeboVisionNode()
    print("Vision Node Started. Waiting for Gazebo camera feed...")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()