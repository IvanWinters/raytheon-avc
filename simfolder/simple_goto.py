"""
Speed test to determine Scout's fastest pace given physical build
and its ability to detect the ArUco marker on the field.

The drone will fly in a straight line and the speeds will be altered.
"""

########DEPENDENCIES###############
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
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
        
def get_current_gps(vehicle):
    location = vehicle.location.global_relative_frame
    return location.lat, location.lon, location.alt
    
def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


def main():
    point = LocationGlobalRelative(38.656653, -77.2433409, 2)
    
    """"
    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not the_connection.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)    
    
    print("Arming motors")
    # Copter should arm in GUIDED mode
    the_connection.mode = VehicleMode("GUIDED")
    while not the_connection.mode == "GUIDED":
        time.sleep(.5)

    the_connection.armed = True

    while not the_connection.armed:      
        print(" Waiting for arming...")
        time.sleep(.5)
    print("Armed motors")
    """

    the_connection.airspeed = 1

    arm_and_takeoff(2)
    the_connection.simple_goto(point)
    
    while get_distance_metres(the_connection.location.global_relative_frame, point) > .5:
        time.sleep(1)
    
    
    
    the_connection.mode = VehicleMode("LAND")

########MAIN#######################
if __name__ == "__main__":

    main()
