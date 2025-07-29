########DEPENDENCIES###############
import json 
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math
import argparse
import logging
import serial
import os
from os import path, sys
import RPi.GPIO as GPIO
from aruco_library import ArucoSingleTracker, load_calibration_data

########CONSTANTS##################
PACKAGE_ALT = 2 # in meters
FREQ_SEND = 2 # seconds
ANGLE_DESCEND_DEG = 20  # degrees
ANGLE_DESCEND_RAD = math.radians(ANGLE_DESCEND_DEG)

############# SET UP LOGGING #############

# Ensure logs directory exists in the current working directory
log_dir = os.path.join(os.getcwd(), "Logs")
if not os.path.exists(log_dir):
    os.makedirs(log_dir)
 
 # Generate a unique log file name based on the current timestamp
log_filename = os.path.join(log_dir, "mission_{}_P.log".format(time.strftime("%Y%m%d_%H%M%S")))
logging.basicConfig(
    filename=log_filename,
    level=logging.INFO,
    filemode='w',
    format='%(asctime)s - %(levelname)s - %(message)s'
)

########CONNECTION BETWEEN FLIGHT COMPUTER AND CONTROLLER#################
parser = argparse.ArgumentParser(description='Demonstrates basic mission operations.')
parser.add_argument('--connect', help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()
 
connection_string = args.connect
 
# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
delivery_drone = connect(connection_string, wait_ready=True)
 
#######CONNECTION BETWEEN SCOUT AND DELIVERY DRONE#########
port = '/dev/ttyUSB0'
ser = serial.Serial(port=port, baudrate=57600, timeout=1)

############# LOAD CALIBRATION & INIT ARUCO TRACKER #############
calibration_filepath = 'calibration_logi.npz'
camera_matrix, dist_coeffs = load_calibration_data(calibration_filepath)
aruco_tracker = ArucoSingleTracker(
    id_to_find=1,       # marker id to detect
    marker_size=0.254,      # marker size in m
    show_video=False,
    camera_matrix=camera_matrix,
    camera_distortion=dist_coeffs
)

########FUNCTIONS##################
# Package Delivery functions
GPIO.setmode(GPIO.BOARD)
GPIO.setup([8,10], GPIO.OUT)
 
GPIO.output(8, GPIO.LOW)
GPIO.output(10, GPIO.LOW)
 
def latch_open():
    GPIO.output(8, GPIO.LOW)
    GPIO.output(10, GPIO.HIGH)
    time.sleep(1)
    GPIO.output(8, GPIO.LOW)
    GPIO.output(10, GPIO.LOW)
 
def latch_close():
    GPIO.output(8, GPIO.HIGH)
    GPIO.output(10, GPIO.LOW)
    time.sleep(1)
    GPIO.output(8, GPIO.LOW)
    GPIO.output(10, GPIO.LOW)

def marker_position_to_angle(x, y, z):
    angle_x = math.atan2(x, z)
    angle_y = math.atan2(y, z)
    return angle_x, angle_y

def check_angle_descend(angle_x, angle_y, angle_desc_rad):
    return math.sqrt(angle_x**2 + angle_y**2) <= angle_desc_rad

def camera_to_uav(x_cam, y_cam):
    """
    Convert camera coordinate system to UAV coordinate system.
    """
    x_uav = -y_cam
    y_uav = x_cam
    return x_uav, y_uav

def uav_to_ne(x_uav, y_uav, yaw_rad):
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)
    north = x_uav * c - y_uav * s
    east  = x_uav * s + y_uav * c
    return north, east

def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    while not delivery_drone.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
    print("Arming motors")
    delivery_drone.mode = VehicleMode("GUIDED")
    delivery_drone.armed = True
    while not delivery_drone.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    print("Taking off!")
    delivery_drone.simple_takeoff(aTargetAltitude)
    while True:
        print(" Altitude: ", delivery_drone.location.global_relative_frame.alt)
        if delivery_drone.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns new latitude and longitude by offsetting the original_location by dNorth and dEast (meters).
    """
    earth_radius = 6378137.0  # meters
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))
    newlat = original_location.lat + (dLat * 180 / math.pi)
    newlon = original_location.lon + (dLon * 180 / math.pi)
    return newlat, newlon

def main():

    #Order of messages
    #Recieve lat long       msg = "{:.7f}, {:.7f}".format(target_location.lat, target_location.lon)
    #Write ack              ack == msg
    #Recieve Mission go     mission_go_msg = "1"

    mission_start_time = time.time()
    logging.info("UXV Start Time: %s", time.ctime(mission_start_time))
  
    while True:
        marker_found, _, _, _ = aruco_tracker.track(loop=False) # precision landing and GPS correction

        mission_go = False # have not received communications yet

        if ser.in_waiting > 0:                                                                              #Message recieved, recieve lat long
            marker_latLong = ser.readline().decode('utf-8').strip()
            
            logging.info(f"Received coordinates: {marker_latLong}")
                        
                        
            marker_lat , marker_long = marker_latLong.split(',')                                            #Tokenize by comma
            marker_lat = marker_lat.strip()                                                                 #Strip of white space
            marker_long = marker_long.strip()
          
            if marker_lat or marker_long:
                logging.info(f"Marker location: Lat={marker_lat}, Long={marker_long}")
                print(f"Marker coordinates Lat : {marker_lat} , Long : {marker_long}")
                ack = "{:.7f}, {:.7f}".format(marker_lat, marker_long)
                ser.write(ack.encode('utf-8'))
                logging.info(f"Sent message: {ack}")
                print(f"Sent message: {ack}")
                
                timeout = time.time() + 10  # Wait up to 10 seconds
                while time.time() < timeout:
                    if ser.in_waiting > 0:                                                                  #Message recieved, wait on mission go
                        msg = ser.readline().decode('utf-8').strip()
                        if msg == "1":
                           mission_go = True 
                           logging.info(f"Received message: {msg}")
                           
                           break
                        else:
                            print(f"Unexpected Message expected mission go , recieved {msg}")               #In case message recieved out of order  
                            logging.warning(f"Unexpected message: {msg}")
                     
                    time.sleep(0.1)
                
                if mission_go == False:
                    print(f"Mission go not recieved before timeout {timeout}")
                    logging.error("Mission go not received before timeout")

                else:
                    print(f"Mission go recieved")
                    logging.info("Mission go received")

                    arm_and_takeoff(PACKAGE_ALT)
                    logging.info("Drone armed and took off")
                    
                    target_location = LocationGlobalRelative(marker_lat, marker_long, PACKAGE_ALT)
                    delivery_drone.simple_goto(target_location)                                             
                    
                    while get_distance_metres(target_location, delivery_drone.location.global_relative_frame) > 1:         
                        time.sleep(0.1)
                    
                    print("Reached marker location")
                    logging.info("Reached marker location, begin drop-off")
                  
        if marker_found:
            print("Marker detected! Initiating Coordinate Calculation.")
            marker_found, x_cam, y_cam, z_cam = aruco_tracker.track(loop=False)
            
            x_uav, y_uav = camera_to_uav(x_cam, y_cam)
            uav_location = delivery_drone.location.global_relative_frame
                
            if uav_location.alt >= 5.0:
                z_cam = uav_location.alt * 100.0
                    
            angle_x, angle_y = marker_position_to_angle(x_uav, y_uav, z_cam)

            if time.time() >= mission_start_time + FREQ_SEND:
                mission_start_time = time.time()

                north, east = uav_to_ne(x_uav, y_uav, delivery_drone.attitude.yaw)
                marker_lat, marker_lon = get_location_metres(uav_location, north, east)

                # Decide whether to descend based on angular error
                if check_angle_descend(angle_x, angle_y, ANGLE_DESCEND_RAD):
                    print("Low angular error: descending")
                    new_alt = max(uav_location.alt - 0.3, 0.5)  # don't go below 0.5m
                else:
                    print("Marker not centered yet: holding altitude")
                    new_alt = uav_location.alt

                target_location = LocationGlobalRelative(marker_lat, marker_lon, new_alt)
                delivery_drone.simple_goto(target_location)

                if (abs(uav_location.lat - marker_lat) <= .000001) and (abs(abs(uav_location.lon) - abs(marker_lon)) <= .000001):
                    print("Centered over marker. Releasing package.")
                    logging.info("Centered over marker. Landing and Releasing package.")
                    logging.info("Landing")
                    delivery_drone.mode = VehicleMode("LAND")
                    while delivery_drone.mode != "LAND":
                        time.sleep(0.2)
                    
                    # Execute package drop
                    latch_open()
                    time.sleep(1)
                    logging.info("Latch opened")

                    logging.info("UXV Mission Complete Time: %s", time.ctime(time.time()))
                    GPIO.cleanup()
                    delivery_drone.close()
                    sys.exit(0)

                    
        elif not marker_found and (mission_go == True): # couldn't find marker, lost in frame, after receiving Scout's message
            print("No marker detected at location . Landing.")

            delivery_drone.mode = VehicleMode("LAND")
            while delivery_drone.mode != "LAND":
                time.sleep(0.2)
                
        

if __name__ == "__main__":
    main()