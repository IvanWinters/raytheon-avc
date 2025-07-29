##Simplified Lightweight version of aruco code for 6x6 markers

import cv2
import numpy as np
import time

#Used to track execution time throughout programa for logging, do:       time.time() - start_time
start_time = time.time()
print("Program start")



# MARKER_SIZE = 0.265 #0.305 #Size of marker (not including white border)
TARGET_ID = 2

#get video feed
cap = cv2.VideoCapture(0)

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)
#Change parameters for higher threshhold
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

#For logging purposes
#file = open("found_markers.txt", 'r').readlines()


while True:
    ret, frame = cap.read()

    if not ret:
        print("Failed to grab frame")
        continue

    #detect markers in the frame
    corners, ids, rejected = detector.detectMarkers(frame)
        

    #Currently detected markers including non-target ones for logging purposes
    detected_markers = []

    #if any markers are detected...
    if ids is not None:
        for i in range(len(ids)):
            #draw the id associated with the detected marker
            cv2.aruco.drawDetectedMarkers(frame, [corners[i]], np.array([[ids[i][0]]], dtype=np.int32))

            #Get the center of the markere based on the average coordinates of the corner
            #This is the center in pixels and not any physical dimension
            x = (corners[i-1][0][0][0] + corners[i-1][0][1][0] + corners[i-1][0][2][0] + corners[i-1][0][3][0]) / 4
            y = (corners[i-1][0][0][1] + corners[i-1][0][1][1] + corners[i-1][0][2][1] + corners[i-1][0][3][1]) / 4
            center = (x,y)


            #for the marker we want
            if ids[i][0] == TARGET_ID:
                #convert marker corners to integer coordinates
                int_corners = np.int32(corners[i])
                #use integer coordinates to draw a red box around marker
                cv2.polylines(frame, [int_corners], isClosed=True, color=(0, 0, 255), thickness=5)
                                
                #print(f"Distance to marker {TARGET_ID}: {distance:.2f} meters")
                #Updated log information for more dynamic logs
                found_target = {
                    "real_time" : time.time(),
                    "program_time" : time.time() - start_time,                    
                    "target_id" : ids[i][0],
                    "center" : center,
                    "is_target" : True
                } 

                #File write debugging
                #First line should change but be static, following lines are appended
                # if file:
                #     file[0] = str(found_target)
                #     file[1] =  "\n"

                # out = open("found_markers.txt", 'w')
                # out.writelines(file)
                # out.close()
                # out = open("found_markers.txt", 'a')
                # out.write(str(found_target))
                
                #Print to STDOUT
                detected_markers.append(found_target)
                
                #draw the coordinate frame axes and print distance onto the frame
                cv2.putText(frame, f"DropZone Found {TARGET_ID}", (10, 70), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
                break
            else: 
                found_target = {
                    "real_time" : time.time(),
                    "program_time" : time.time() - start_time,                    
                    "target_id" : ids[i][0],
                    "center" : center,
                    "is_target" : False
                } 
                cv2.putText(frame, "Non-DropZone(s)", (10, 110), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)  
                
                detected_markers.append(found_target)


        print(detected_markers)
                          

    cv2.imshow('frame', frame)

    if cv2.waitKey(1) == ord('q'):
        print("SHUTDOWN DETECTED")
        break

cap.release()
cv2.destroyAllWindows()
