from pymavlink import mavutil 
import time

#Creates mavlink connection to receive so that scout drone can listen for incoming messages
uav1 = mavutil.mavlink_connection('udp:127.0.0.1:14551')

#Creates mavlink connection to send so that messages can be delivered to delivery
uav2_connection = mavutil.mavlink_connection('udpout:127.0.0.1:14552')

uav1.wait_heartbeat()
print("UAV1: Heartbeat received") 

#Function to send simple status text
def send_custom_message():
    uav2_connection.mav.statustext_send(
        mavutil.mavlink.MAV_SEVERITY_NOTICE,
        "Hello from UAV1".encode()
    )
    print("Custom message sent to UAV2")

send_custom_message()

#Receives its own gps location and returns its latitude, longitude, and altitude 
def get_gps():
    msg = uav1.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    lat = msg.lat / 1e7
    lon = msg.lon / 1e7
    alt = msg.alt / 1000
    return lat, lon, alt

lat, lon, alt = get_gps()
print(f"Sending GPS to UAV2 - Latitude: {lat}, Longitude: {lon}, Altitude: {alt}")

#Sends gps location through uav2 connection 
uav2_connection.mav.statustext_send(
    mavutil.mavlink.MAV_SEVERITY_NOTICE,
    f"GPS:{lat}:{lon}:{alt}".encode()
)
print("GPS data sent to UAV2")