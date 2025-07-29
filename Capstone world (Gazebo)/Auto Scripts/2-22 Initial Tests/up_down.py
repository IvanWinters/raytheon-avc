"""
February 22, 2025

The Drone Kit library will be used to make the drone go up 1 meter AGL, hover for a short time, and land back down.
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

def main():
    print('Up Down')

    arm_and_takeoff(2) #fly 2 meter up
    time.sleep(10) # hold position for some time
  
    the_connection.mode = VehicleMode("LAND")
    the_connection.close()
    
########MAIN#######################
if __name__ == "__main__":

    main()
