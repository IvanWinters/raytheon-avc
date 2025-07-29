########DEPENDENCIES###############
import json 
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
import time
import math
import argparse
import logging
import serial
import os
import socket
from os import path, sys
#from gpiozero import Motor
#from drop import open as drop_open
#from drop import close as drop_close


########CONSTANTS##################
PACKAGE_ALT = 1 # in meters
MAX_AIRSPEED = 1

############# TELEMETRY RADIO CONNECTION #############
#PORT = '/dev/ttyUSB0'
#BAUDRATE=57600
#TIMEOUT=1

############# SET UP LOGGING #############
# Ensure logs directory exists in the current working directory
log_dir = os.path.join(os.getcwd(), "Logpd")
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

############# CONNECTION #############
parser = argparse.ArgumentParser(description='Scout Challenge 2 Mission')
parser.add_argument('--connect', help="Vehicle connection target string.", default='')
args = parser.parse_args()
connection_string = args.connect

print("Connecting to vehicle on: %s" % connection_string)
delivery_drone = connect(connection_string, wait_ready=True)

#Socket connection established
HOST = '127.0.0.1'
PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((HOST, PORT))

########FUNCTIONS##################
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

############# MAIN METHOD #############

def main():
    #set up logging
    log_dir = os.path.join(os.getcwd(), "Logs")
    os.makedirs(log_dir, exist_ok=True)
    log_filename = os.path.join(log_dir, f"mission_{time.strftime('%Y%m%d_%H%M%S')}_P.log")
    logging.basicConfig(filename=log_filename,
                        level=logging.INFO,
                        filemode='w',
                        format='%(asctime)s - %(levelname)s - %(message)s')

    #open serial port
    #ser = serial.Serial(port=PORT, baudrate=BAUDRATE, timeout=TIMEOUT)
    logging.info("UXV connected—waiting for mission authorization")

    while True:
        msg, addr = sock.recvfrom(1024)
        line = msg.decode().strip()
        print(f"Received: {line}")

        try:
            lat_str, lon_str = line.split(',')
            target_lat = float(lat_str)
            target_lon = float(lon_str)
        except:
            continue

        ack = f"{target_lat:.7f},{target_lon:.7f}".encode()
        sock.sendto(ack, addr)

        #Wait for 'authorized'
        msg, _ = sock.recvfrom(1024)
        if msg.decode().strip() != "authorized":
            continue

        #Mission logic
        point = LocationGlobalRelative(target_lat, target_lon, PACKAGE_ALT)
        delivery_drone.airspeed = MAX_AIRSPEED
        arm_and_takeoff(2)
        delivery_drone.simple_goto(point)
        while get_distance_metres(delivery_drone.location.global_relative_frame, point) > 0.5:
            time.sleep(1)

        #Simulate drop
        print("Simulating drop package")
        time.sleep(2)
        #drop_open()
        print("Package released")
        break

if __name__ == "__main__":
    main()
