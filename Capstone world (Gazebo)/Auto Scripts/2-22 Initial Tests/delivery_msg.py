"""
February 22, 2025
Test Python sockets on the prototype drone.
The prototype will act as Delivery drone, and a separate flight computer
will send commands, and not the GCS like the Scout drone will.

This implementation has the Scout sending all actions: Arming, Disarming,
Takeoff, Land, and Move. It also only uses pymavlink
"""

import socket
import json 
import threading
from pymavlink import mavutil

#Creating a connection to the "Delivery drone" using mavlink
uav = mavutil.mavlink_connection('udp:127.0.0.1:14550')
uav.wait_heartbeat()
print("Delivery drone connected and ready.")

#Also sets up a UDP socket for communication indicating both the host and port
HOST = '127.0.0.1'
PORT = 5001
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_socket.bind((HOST, PORT))

def execute_command(command):
    #All executable comands that uav1 will listen for
    parts = command.split()
    if parts[0] == "arm":
        print("Arming Delivery drone...")
        uav.mav.command_long_send(
            uav.target_system, uav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0
        )
    elif parts[0] == "disarm":
        print("Disarming Delivery drone...")
        uav.mav.command_long_send(
            uav.target_system, uav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0
        )
    elif parts[0] == "takeoff":
        altitude = float(parts[1])
        print(f"Taking off to {altitude} meters...")
        uav.mav.command_long_send(
            uav.target_system, uav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, altitude
        )
    elif parts[0] == "land":
        print("Landing Delivery drone...")
        uav.mav.command_long_send(
            uav.target_system, uav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0
        )
    elif parts[0] == "move":
        lat, lon, alt = map(float, parts[1:])
        print(f"Moving Delivery drone to ({lat}, {lon}, {alt})...")
        uav.mav.set_position_target_global_int_send(
            0, uav.target_system, uav.target_component, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            int(0b110111111000), int(lat * 1e7), int(lon * 1e7), alt,
            0, 0, 0, 0, 0, 0, 0, 0
        )

def listen_for_messages():
    #UAV1 will constantly be listening for messages from UAV2
    while True:
        data, addr = server_socket.recvfrom(1024)
        command = data.decode()
        print(f"Received command: {command} from {addr}")
        execute_command(command)

listener_thread = threading.Thread(target=listen_for_messages, daemon=True)
listener_thread.start()

print("Delivery drone is waiting for commands...")

while True:
    msg = uav.recv_match(blocking=True)
    if msg:
        print(f"Delivery drone received MAVLink message: {msg}")
