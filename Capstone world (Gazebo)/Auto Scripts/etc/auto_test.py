"""
Solely using MAVLINK messages to generate auto mission
"""

from pymavlink import mavutil

# Start a connection listening on a UDP port
the_connection = mavutil.mavlink_connection('udpin:localhost:14551')

# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

# Once connected, use 'the_connection' to get and send messages

the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)

the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10)

msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)
"""
from pymavlink import mavutil


import time
 
def connect_to_vehicle(connection_str):

   
    print(f"Connecting to vehicle on {connection_str}")

    vehicle = mavutil.mavlink_connection(connection_str)

    vehicle.wait_heartbeat()

    print("Heartbeat received from vehicle")
    return vehicle
 
def set_mode(vehicle, mode):

    #Set the flight mode of the vehicle.

    vehicle.mav.command_long_send(

        vehicle.target_system,

        vehicle.target_component,

        mavutil.mavlink.MAV_CMD_DO_SET_MODE,

        0,

        1,  # Mode ID (1 for AUTO)

        0, 0, 0, 0, 0, 0

    )

    print(f"Mode set to {mode}")
 
def arm_and_takeoff(vehicle, altitude):

    #Arm the vehicle and take off to the specified altitude.

    print("Arming the vehicle...")

    vehicle.mav.command_long_send(

        vehicle.target_system,

        vehicle.target_component,

        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,

        0,

        1, 0, 0, 0, 0, 0, 0

    )

    print("Waiting for the vehicle to arm")

    while not vehicle.motors_armed():

        time.sleep(1)

    print("Taking off...")

    vehicle.mav.command_long_send(

        vehicle.target_system,

        vehicle.target_component,

        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,

        0,

        0, 0, 0, 0, 0, altitude

    )
 
def create_waypoint(lat, lon, alt):

    #Create a MAVLink waypoint message.

    return mavutil.mavlink.MAVLink_mission_item_message(

        0, 0, 0,

        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,

        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,

        0, 0, 0, 0, 0, 0,

        lat, lon, alt

    )
 
def upload_mission(vehicle, waypoints):

    #Upload the mission waypoints to the vehicle.

    print("Uploading mission...")

    vehicle.waypoint_clear_all_send()

    vehicle.waypoint_count_send(len(waypoints))

    for i, wp in enumerate(waypoints):

        vehicle.mav.send(wp)

        print(f"Waypoint {i+1} sent")
 
def main():

    connection_str = "tcp:127.0.0.1:5501"  # SITL connection

    vehicle = connect_to_vehicle(connection_str)

    # Define waypoints (latitude, longitude, altitude in meters)

    waypoints = [

        create_waypoint(37.427631, -122.169708, 10),  # Waypoint 1

        create_waypoint(37.428101, -122.168911, 15),  # Waypoint 2

        create_waypoint(37.428501, -122.170411, 10),  # Waypoint 3

    ]

    upload_mission(vehicle, waypoints)

    set_mode(vehicle, "AUTO")

    arm_and_takeoff(vehicle, 10)
 
    print("Mission in progress...")

    while True:

        # Monitor mission progress (e.g., check for mission completion)

        msg = vehicle.recv_match(type="MISSION_ITEM_REACHED", blocking=True)

        print(f"Reached waypoint {msg.seq}")

        if msg.seq == len(waypoints) - 1:

            print("Mission completed")

            break
 
    print("Landing...")

    vehicle.mav.command_long_send(

        vehicle.target_system,

        vehicle.target_component,

        mavutil.mavlink.MAV_CMD_NAV_LAND,

        0,

        0, 0, 0, 0, 0, 0

    )
 
if __name__ == "__main__":

    main()

 """
