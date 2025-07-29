import cv2
import numpy as np
import pymavlink.mavutil as mavutil
from cv2 import aruco

# Connect to the drone
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat()
print("Connected to drone")

# ArUco dictionary and parameters
dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(dictionary, parameters)

# Camera setup
cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        continue
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)
    
    if ids is not None:
        for i in range(len(ids)):
            c = corners[i][0]
            center_x = int((c[0][0] + c[2][0]) / 2)
            center_y = int((c[0][1] + c[2][1]) / 2)
            
            # Draw marker
            cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)
            cv2.putText(frame, f"ID: {ids[i][0]}", (center_x, center_y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            
            # Send drone position adjustment commands
            master.mav.set_position_target_local_ned_send(
                0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                int(0b110111111000), 0, 0, -2, 0, 0, 0, 0, 0, 0, 0, 0)
            
    cv2.imshow("Aruco Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
