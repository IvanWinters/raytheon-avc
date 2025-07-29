"""
February 22, 2025

Speed test to determine Scout's fastest pace given physical build
and its ability to detect the ArUco marker on the field.
 
The drone will fly in a straight line and the speeds will be altered, while scanning for markers.
"""
 
########DEPENDENCIES###############
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math
import argparse
from pymavlink import mavutil

# pip install opencv-contrib-python
import cv2
import numpy as np
 
#Used to track execution time throughout programa for logging, do:       time.time() - start_time
start_time = time.time()
 
########CONNECTION#################
parser = argparse.ArgumentParser(description='Demonstrates basic mission operations.')
parser.add_argument('--connect',
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()
 
connection_string = args.connect
 
# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
the_connection = connect(connection_string, wait_ready=True)
 
########FUNCTIONS##################
 
#Send a velocity command with +x being the heading of the drone.
def send_local_ned_velocity(vx, vy, vz):
    msg = the_connection.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0)
    the_connection.send_mavlink(msg)
    the_connection.flush()
 
def arm_and_takeoff(aTargetAltitude):
 
    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not the_connection.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
 
       
    print("Arming motors")
    # Copter should arm in GUIDED mode
    the_connection.mode = VehicleMode("GUIDED")
    the_connection.armed = True
 
    while not the_connection.armed:      
        print(" Waiting for arming...")
        time.sleep(1)
 
    print("Taking off!")
    the_connection.simple_takeoff(aTargetAltitude) # Take off to target altitude
 
    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", the_connection.location.global_relative_frame.alt)      
        if the_connection.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)
 
 
#Added counter variable to see what time the drone is at in the main function
def aruco_detection(counter):
    start_detection_time = time.time()      #used to keep track of when to exit function
 
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    parameters = cv2.aruco.DetectorParameters()
    cap = cv2.VideoCapture(0)       #May need to change param nubmer depending on camera options
 
    #Will run this while loop for how ever many seconds put in function parameter
    while (time.time() - start_detection_time) < counter :
        ret, frame = cap.read()
 
        if not ret:
            print("Failed to grab frame")       #Meaning the camera couldnt be used/funciton, usually due to the wrong VideoCapture being selected
            return
        
        corners, ids, rejected = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

        send_local_ned_velocity(2, 0, 0) # fly 2m/s forward while scanning
     
        #Aruco found
        if ids is not None:
            found_target = {
                "real_time" : time.time(),                                  #Real world time
                "program_time" : time.time() - start_time,                  #Time since program start (approx.)
                "detection time" : counter,                                 #Refers to the time since this function start
                "speed" :  the_connection.groundspeed,                      #Speed when marker detected
                "target_id" : ids[0][0],
            }
 
            print("ARUCO DETECTED")
            print(found_target)
            
            cap.release()
            cv2.destroyAllWindows()
            return True
        
    print("NO MARKER")
    the_connection.mode = VehicleMode("RTL") #if unable to detect ArUco after counter stops, RTL
 
    cap.release()
    cv2.destroyAllWindows()
 
      
def main():
    print('Speed vs. ArUco Detection')
    arm_and_takeoff(1) #fly 1 meter up
 
    #Run aruco detection for 5 seconds, nested velocity command for simultaneous flying while scanning
    #Drone will fo forward 10 meters (approx. 33 feet) total if no marker was found in between time
    aruco_detection(5)
       
    the_connection.close()
  
########MAIN#######################
if __name__ == "__main__":
 
    main()
