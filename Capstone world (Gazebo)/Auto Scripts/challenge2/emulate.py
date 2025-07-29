import serial
import time
import logging

def communicate(target_location, port, max_attempts=3):
    msg = "{:.7f}, {:.7f}".format(target_location[0], target_location[1])
    
    try:
        ser = serial.Serial(port=port, baudrate=57600, timeout=1)
    except serial.SerialException as e:
        logging.error("Failed to open serial port: %s", e)
        return
    
    attempt = 0
    ack_received = False

    while attempt < max_attempts and not ack_received:
        attempt += 1
        # Send the target location message to the receiver
        ser.write(msg.encode('utf-8'))
        logging.info(
            "COMMS Transmitted the Marker Location: Lat = %.7f, Lon = %.7f (Attempt %d)",
            target_location[0], target_location[1], attempt
        )
        
        # Wait for acknowledgment from the second device
        timeout = time.time() + 5
        while time.time() < timeout:
            if ser.in_waiting > 0:
                ack = ser.readline().decode('utf-8').strip()
                if ack == msg:
                    ack_received = True
                    logging.info("Received ACK from second device.")
                    print("Received ACK from second device.")
                    break
                else:
                    logging.warning("Received incorrect ACK: '%s' (expected '%s')", ack, msg)
                    break
            time.sleep(0.1)
        
        if not ack_received:
            logging.warning("ACK not received or incorrect, resending message (Attempt %d)", attempt)
    
    if ack_received:
        mission_go_msg = '1'
        try:
            ser.write(mission_go_msg.encode('utf-8'))
            logging.info("COMMS Transmitted 'mission go' message to second device")
            print("Sent 'mission go' message to second device")
        except serial.SerialException as e:
            logging.error("Failed to send 'mission go' message: %s", e)

    else:
        logging.error("Failed to receive correct ACK after %d attempts.", max_attempts)
    
    ser.close()

def main():
    target_location = (38.6779195, -77.2493720)
    port = '/dev/ttyUSB0'
    communicate(target_location, port)

if __name__ == "__main__":
    main()
