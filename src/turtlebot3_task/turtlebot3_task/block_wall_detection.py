#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np

class GazeboVisionNode(Node):
    def __init__(self):
        super().__init__('gazebo_vision_node')
        self.bridge = CvBridge()
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)

    def get_vision_data(self, frame):
        data = {"color": None, "current_marker_id": None, "too_close": False, "center_x": None}
        h, w, _ = frame.shape
        
        # Wall Marker Detection
        corners, ids, _ = self.detector.detectMarkers(frame)
        if ids is not None:
            data["current_marker_id"] = int(ids[0][0])
            width = np.linalg.norm(corners[0][0][0] - corners[0][0][1])
            if width > 145: data["too_close"] = True

        # Block Detection
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_r = cv2.addWeighted(cv2.inRange(hsv, np.array([0,150,50]), np.array([10,255,255])), 1.0,
                                 cv2.inRange(hsv, np.array([160,150,50]), np.array([180,255,255])), 1.0, 0.0)
        mask_b = cv2.inRange(hsv, np.array([110,150,50]), np.array([125,255,255]))
        
        mask_r[0:int(h*0.5), :] = 0
        mask_b[0:int(h*0.5), :] = 0

        for color, mask in [('red', mask_r), ('blue', mask_b)]:
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if cnts:
                c = max(cnts, key=cv2.contourArea)
                if cv2.contourArea(c) > 500:
                    M = cv2.moments(c)
                    data["color"] = color
                    data["center_x"] = int(M["m10"] / M["m00"])
                    break
        return data