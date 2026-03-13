import cv2
import cv2.aruco as aruco
import numpy as np
import sys

print("--- Starting Robot Master Vision ---")

# --- 1. SETUP ARUCO (Version Compatible) ---
try:
    # Modern OpenCV (4.7.0+)
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, parameters)
    print("Using Modern ArUco Detector")
except AttributeError:
    # Legacy OpenCV
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters_create()
    detector = None
    print("Using Legacy ArUco Detector")

# --- 2. SETUP CAMERA ---
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("ERROR: Could not open camera.")
    sys.exit()

print("SUCCESS: Camera active. Press 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert to HSV for color detection
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # --- 3. COLOR DETECTION: BLUE (Your exact numbers) ---
    lower_blue = np.array([110, 150, 50]) 
    upper_blue = np.array([125, 255, 255])
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

    # --- 4. COLOR DETECTION: RED (Your exact numbers) ---
    lower_red = np.array([165, 150, 70]) 
    upper_red = np.array([180, 255, 255])
    mask_red = cv2.inRange(hsv, lower_red, upper_red)

    # --- 5. ARUCO DETECTION: WALLS (Version Compatible) ---
    if detector:
        # Modern Way
        corners, ids, rejected = detector.detectMarkers(frame)
    else:
        # Legacy Way
        corners, ids, rejected = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

    # Label markers if found
    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)
        for i, marker_id in enumerate(ids):
            cv2.putText(frame, f"WALL ID: {marker_id[0]}", (10, 50 + i*35), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

    # --- 6. TARGETING: DRAW BOXES ON CUBES ---
    # Blue Contours
    blue_cnts, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in blue_cnts:
        if cv2.contourArea(cnt) > 2000:
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
            cv2.putText(frame, "BLUE CUBE", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

    # Red Contours
    red_cnts, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in red_cnts:
        if cv2.contourArea(cnt) > 2000:
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
            cv2.putText(frame, "RED CUBE", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    # --- 7. DISPLAY WINDOWS ---
    cv2.imshow('Robot Master Vision', frame)
    cv2.imshow('Red Mask', mask_red)
    cv2.imshow('Blue Mask', mask_blue)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
