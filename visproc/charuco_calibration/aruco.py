import cv2
import numpy as np

calibration_data = np.load('calibration_data_live.npz')
camera_matrix = calibration_data['camera_matrix']
dist_coeffs = calibration_data['dist_coeffs']

ARUCO_DICT = {
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}

#14 inch = .3556 m
MARKER_SIZE = 0.305
TARGET_ID = 17

#get video feed
cap = cv2.VideoCapture(0)

arucoType = None

#default parameters for ArUco marker detection
parameters = cv2.aruco.DetectorParameters()

while True:
    ret, frame = cap.read()

    if not ret:
        print("Failed to grab frame")
        continue

    #get all the dict from aruco_dict list
    if arucoType is None:
        #this is not optimized for using just 1 dict
        for (arucoName, arucoDict) in ARUCO_DICT.items():
            arucoDict = cv2.aruco.getPredefinedDictionary(arucoDict)
            #corners holds the corners of detected markers
            #ids contains the IDs of the detected markers
            #rejected holds any contours detected as markers that don’t match a marker ID
            (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, arucoDict, parameters=parameters)
            if ids is not None:
                arucoType = arucoDict
                break
    else:
        #detect markers in the frame
        corners, ids, rejected = cv2.aruco.detectMarkers(frame, arucoType, parameters=parameters)

    #if any markers are detected...
    if ids is not None:
        for i in range(len(ids)):
            #draw the id associated with the detected marker
            cv2.aruco.drawDetectedMarkers(frame, [corners[i]], np.array([[ids[i][0]]], dtype=np.int32))

            #for the marker we want
            if ids[i][0] == TARGET_ID:
                #convert marker corners to integer coordinates
                int_corners = np.int32(corners[i])
                #use integer coordinates to draw a red box around marker
                cv2.polylines(frame, [int_corners], isClosed=True, color=(0, 0, 255), thickness=5)
                
                #estimate the pose of target marker
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], MARKER_SIZE, camera_matrix, dist_coeffs)

                #calculate and print the distance to console
                distance = np.linalg.norm(tvec)
                print(f"Distance to marker {TARGET_ID}: {distance:.2f} meters")
                
                #draw the coordinate frame axes and print distance onto the frame
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.1)
                cv2.putText(frame, f"Distance: {distance:.2f}m", (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

    cv2.imshow('frame', frame)

    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()