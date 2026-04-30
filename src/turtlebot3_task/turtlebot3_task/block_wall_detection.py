#!/usr/bin/env python3
import cv2
import cv2.aruco as aruco
import numpy as np

# HSV ranges tuned for Gazebo Classic rendering
_RED_LO1  = np.array([0,   150, 50])
_RED_HI1  = np.array([10,  255, 255])
_RED_LO2  = np.array([160, 150, 50])
_RED_HI2  = np.array([180, 255, 255])
_BLUE_LO  = np.array([100, 150, 50])
_BLUE_HI  = np.array([130, 255, 255])

# TIGHTENED: Lower area threshold to catch blocks at a distance
_MIN_BLOCK_AREA = 400 

# ArUco marker width (pixels) above which the robot is "too close"
_TOO_CLOSE_WIDTH = 140

class VisionHelper:
    def __init__(self):
        aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        params      = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(aruco_dict, params)

    def analyse(self, frame):
        h, w = frame.shape[:2]
        result = {
            'marker_id': None,
            'too_close':  False,
            'color':      None,
            'center_x':   None,
            'frame_w':    w,
        }

        # ArUco Detection
        corners, ids, _ = self.detector.detectMarkers(frame)
        if ids is not None:
            result['marker_id'] = int(ids[0][0])
            width_px = float(np.linalg.norm(corners[0][0][0] - corners[0][0][1]))
            result['too_close'] = width_px > _TOO_CLOSE_WIDTH

        # Colour detection (lower half only to ignore wall markers)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        half = h // 2

        mask_r = cv2.add(
            cv2.inRange(hsv, _RED_LO1, _RED_HI1),
            cv2.inRange(hsv, _RED_LO2, _RED_HI2))
        mask_b = cv2.inRange(hsv, _BLUE_LO, _BLUE_HI)

        for mask in (mask_r, mask_b):
            mask[:half, :] = 0

        for color, mask in (('red', mask_r), ('blue', mask_b)):
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not cnts:
                continue
            best = max(cnts, key=cv2.contourArea)
            if cv2.contourArea(best) < _MIN_BLOCK_AREA:
                continue
            M = cv2.moments(best)
            if M['m00'] == 0:
                continue
            result['color']    = color
            result['center_x'] = int(M['m10'] / M['m00'])
            break   

        return result