"""
Search Mission with Precise Landing for CHALLENGE 1

This script performs a search pattern mission. While the drone is
following the search waypoints, it monitors for an ArUco marker. Once a marker
is detected, the vehicle transitions to a precise landing routine guided by the marker.
"""

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from pymavlink import mavutil
import time
import numpy as np
import math
import argparse
import os
from os import path, sys
from aruco_library import ArucoSingleTracker, load_calibration_data
# from new_aruco_lib import ArucoSingleTracker, load_calibration_data
import logging
import serial

############# SET UP LOGGING #############
# Ensure logs directory exists in the current working directory
log_dir = os.path.join(os.getcwd(), "Logs")
if not os.path.exists(log_dir):
    os.makedirs(log_dir)
    
# Generate a unique log file name based on the current timestamp
log_filename = os.path.join(log_dir, "mission_{}.log".format(time.strftime("%Y%m%d_%H%M%S")))
logging.basicConfig(
    filename=log_filename,
    level=logging.INFO,
    filemode='w',
    format='%(asctime)s - %(levelname)s - %(message)s'
)


############# TELEMETRY RADIO CONNECTION #############
PORT = '/dev/ttyUSB0'
BAUDRATE = 57600
TIMEOUT = 1

############# CONSTANTS #############
# Search parameters
pi = math.pi
T_STEP = pi/8           # granularity of curve (time step)
T_MAX = 64              # number of time points for the mission
AMP_X = 8               # meters, half-amplitude in X-direction
AMP_Y = 8               # meters, half-amplitude in Y-direction
W_X = 1                 # angular frequency in X-direction
W_Y = math.sqrt(2)      # angular frequency in Y-direction
PHI_X = pi/2            # phase shift in X-direction
PHI_Y = 0               # phase shift in Y-direction
SCOUT_ALT = 4           # scouting altitude (meters)
CENTER_LAT = 11.111111 #CHANGE ME
CENTER_LONG = 11.111111 #CHANGE ME
EARTH_RADIUS = 6378.137   # km

############# CONNECTION #############
parser = argparse.ArgumentParser(description='Scout Challenge 1 Mission')
parser.add_argument('--connect', help="Vehicle connection target string.", default='')
args = parser.parse_args()
connection_string = args.connect

print("Connecting to vehicle on: %s" % connection_string)
scout = connect(connection_string, wait_ready=True)

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

############# HELPER FUNCTIONS #############
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

def marker_position_to_angle(x, y, z):
    angle_x = math.atan2(x, z)
    angle_y = math.atan2(y, z)
    return angle_x, angle_y

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

def check_angle_descend(angle_x, angle_y, angle_desc):
    return math.sqrt(angle_x**2 + angle_y**2) <= angle_desc

def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    while not scout.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
    print("Arming motors")
    scout.mode = VehicleMode("GUIDED")
    scout.armed = True
    while not scout.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    print("Taking off!")
    scout.simple_takeoff(aTargetAltitude)
    while True:
        print(" Altitude: ", scout.location.global_relative_frame.alt)
        if scout.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)
    #scout.airspeed = 5 # adjust as needed

############# MISSION FUNCTIONS #############
def search():
    print("Uploading search pattern")
    cmds = scout.commands
    cmds.download()    # Download existing commands
    cmds.wait_ready()
    print("Clearing existing commands")
    cmds.clear()
    rows = 2 * AMP_Y + 1
    columns = 2 * AMP_X + 1
    met_to_deg = (1 / ((pi / 180) * EARTH_RADIUS)) / 1000  # conversion constant
    latitude_arr = np.zeros((rows, columns), dtype=np.float64)
    longitude_arr = np.zeros((rows, columns), dtype=np.float64)
    for row in range(rows):
        for col in range(columns):
            latitude_arr[row][col] = CENTER_LAT + (AMP_Y - row) * met_to_deg
            longitude_arr[row][col] = CENTER_LONG + ((AMP_X - col) * met_to_deg) / (math.cos(CENTER_LAT * (pi/180)))
    # Add MAV_CMD_NAV_TAKEOFF command (ignored if already airborne)
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, SCOUT_ALT))
    # Add waypoints
    for T_RANGE in range(0, T_MAX):
        X_point = math.ceil(AMP_X * math.sin(W_X * T_STEP * T_RANGE + PHI_X))
        Y_point = math.ceil(AMP_Y * math.sin(W_Y * T_STEP * T_RANGE + PHI_Y))
        lat = latitude_arr[-Y_point + AMP_Y][X_point + AMP_X]
        lon = longitude_arr[-Y_point + AMP_Y][X_point + AMP_X]
        cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                         mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
                         lat, lon, SCOUT_ALT))
    print("Search waypoints uploaded.")
    cmds.upload()
    


def wait_for_mode(vehicle, mode, timeout=10):
    start = time.time()
    vehicle.mode = VehicleMode(mode)
    while vehicle.mode.name != mode and time.time() - start < timeout:
        time.sleep(0.2)
    return vehicle.mode.name == mode


def send_marker_to_pd(lat, lon, ser):
    msg = f"{lat:.7f}, {lon:.7f}"
    ser.write(msg.encode('utf-8'))
    logging.info("COMMS transmitted: %s", msg)

    
def handle_marker_sequence(vehicle, aruco_tracker, ser):
    # Switch to GUIDED once
    if not wait_for_mode(vehicle, 'GUIDED'):
        logging.error("Failed to enter GUIDED mode")
        return

    while True:
        found, x_cam, y_cam, z_cam = aruco_tracker.track(loop=False)
        if not found:
            # Lost marker: resume search
            logging.info("Lost marker, resuming AUTO")
            vehicle.mode = VehicleMode('AUTO')
            wait_for_mode(vehicle, 'AUTO')
            return

        # Compute real-world coords
        x_uav, y_uav = camera_to_uav(x_cam, y_cam)
        uav_loc = vehicle.location.global_relative_frame
        north, east = uav_to_ne(x_uav, y_uav, vehicle.attitude.yaw)
        lat, lon = get_location_metres(uav_loc, north, east)
        logging.info("Marker at %.7f, %.7f", lat, lon)

        # Communicate and land
        send_marker_to_pd(lat, lon, ser)

        if wait_for_mode(vehicle, 'LAND'):
            logging.info("Landing initiated")
        else:
            logging.error("Failed to enter LAND mode")
        logging.info("Mission end time: %s", time.ctime())
        vehicle.close()
        sys.exit(0)
    


def main():
    ser = serial.Serial(port=PORT, baudrate=BAUDRATE, timeout=TIMEOUT)
    search()
    logging.info("UAV start time: %s", time.ctime())

    arm_and_takeoff(SCOUT_ALT)
    logging.info("Starting search mission")

    # Enter AUTO
    next_wp = 0
    scout.commands.next = next_wp
    wait_for_mode(scout, 'AUTO')


    first_detect = False
    while True:
        current_wp = scout.commands.next
        found, *_ = aruco_tracker.track(loop=False)

        if found:
            if not first_detect:
                logging.info("Marker detected at %s", time.ctime())
                first_detect = True
            handle_marker_sequence(scout, aruco_tracker, ser)

        if current_wp >= T_MAX:
            logging.info("No marker found, returning to launch")
            scout.mode = VehicleMode('RTL')
            break

        time.sleep(1)

    logging.info("Mission complete at %s", time.ctime())
    scout.close()


if __name__ == '__main__':
    main()
