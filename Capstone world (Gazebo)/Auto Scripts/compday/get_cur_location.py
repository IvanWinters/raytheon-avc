"""
Speed test to determine Scout's fastest pace given physical build
and its ability to detect the ArUco marker on the field.

The drone will fly in a straight line and the speeds will be altered.
"""

########DEPENDENCIES###############
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math
import argparse
from pymavlink import mavutil

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
def get_current_gps(vehicle):
    location = vehicle.location.global_relative_frame
    return location.lat, location.lon, location.alt


def main():
    lat, lon, alt = get_current_gps(the_connection)
    print("Current GPS:", lat, lon, alt)

########MAIN#######################
if __name__ == "__main__":

    main()
