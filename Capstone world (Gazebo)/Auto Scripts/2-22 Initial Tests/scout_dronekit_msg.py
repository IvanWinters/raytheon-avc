"""
February 22, 2025
Test Python sockets on the prototype drone.
The prototype will act as Delivery drone, and a separate flight computer
will send commands, and not the GCS like the Scout drone will.

This implementation has the Scout send actions after Delivery drone
arms and takes off itself.

There is no second prototype, so the Scout can have sockets just for now.
"""

import socket
import time
import random
import re
from pymavlink import mavutil

#Sets up UDP socket for sending messages
HOST = '127.0.0.1'
PORT = 5001
client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_command(command):
    client_socket.sendto(command.encode(), (HOST, PORT))
    print(f"Sent command to Delivery drone: {command}")

print("Scout is starting...")

#Simulated flight operations
"""
send_command("arm")
time.sleep(8)
send_command("takeoff 10")
time.sleep(15)
send_command("move 47.397748 8.545596 20") # Change GPS coordinates on field

#Logs that mimick how the visual processing entries will end up looking like
def generate_fake_logs():
    events = ["Centered", "Misaligned"]
    while True:
        event = random.choice(events)
        roll_angle = round(random.uniform(170, 190), 2)
        log_entry = f'{{"event": "{event}", "roll_angle": {roll_angle}, "marker_center_x": 314, "marker_center_y": 244}}'
        print(f"Log Entry: {log_entry}")  
        yield log_entry
        time.sleep(1) 

#Function to monitor logs and check conditions
def monitor_logs():
    for log_entry in generate_fake_logs():
        match_event = re.search(r'"event":\s*"(\w+)"', log_entry)
        match_roll = re.search(r'"roll_angle":\s*([\d.]+)', log_entry)

        if match_event and match_roll:
            event = match_event.group(1)
            roll_angle = float(match_roll.group(1))

            if event == "Centered" and 175 <= roll_angle <= 185:
                print("Condition met: Sending LAND command to Delivery Drone.")
                send_command("land")
                wait_for_landing()
                break  
"""
#Function to wait until Delivery Drone has taken off before sending location
def wait_to_send():
    print("Waiting for Delivery Drone to land...")
    while True:
        msg = uav.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
        if msg:
            altitude = msg.relative_alt / 1000.0 
            print(f"Delivery Drone Altitude: {altitude:.2f}m")

            if(altitude <= 1*.95 | altitude >= 1*.95) # reached target altitude +/- some variation
                send_command("move 47.397748 8.545596 20") # Change coordinates at field, later this can be scout.location() 
            
            """
            if altitude <= 0.1:  
                print("Delivery Drone has landed. Sending DISARM command.")
                #This will also be when the Scout's work is done, so send it an RTL
                send_command("disarm")
                break
            """
        time.sleep(1)

#monitor_logs()
