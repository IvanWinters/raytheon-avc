import time
from pymavlink import mavutil

#Creates mavlink connection to receive 
uav2 = mavutil.mavlink_connection('udp:127.0.0.1:14552')
uav2.wait_heartbeat()

print("UAV2: Heartbeat received")

#Sets up receiver so that it can receive status text
msg = uav2.recv_match(type='STATUSTEXT', blocking=True)
if msg:
    print(f"UAV2: Received message - {msg}")

#Function to change the mode of the delivery uav 
def set_mode(mode):
    mode_id = uav2.mode_mapping()[mode]
    uav2.mav.set_mode_send(
        uav2.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

#Commits arm and takeoff commands 
def arm_and_takeoff(altitude):
    print("Arming Delivery")
    uav2.mav.command_long_send(
        uav2.target_system,
        uav2.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    time.sleep(2)
    
    print(f"Taking off to {altitude} meters")
    uav2.mav.command_long_send(
        uav2.target_system,
        uav2.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0, altitude
    )

while True:
    msg1 = uav2.recv_match(type='STATUSTEXT', blocking=True)
    text = msg1.text
    
    #Checks if gps location is received, parses the lat, lon, alt, and commands delivery uav to takeoff
    if text.startswith("GPS:"):
        _, lat, lon, alt = text.split(":")
        lat, lon, alt = float(lat), float(lon), float(alt)
        print(f"Received GPS - Latitude: {lat}, Longitude: {lon}, Altitude: {alt}")
        set_mode("GUIDED")
        print("Switched to GUIDED mode")
        arm_and_takeoff(alt)
        break
