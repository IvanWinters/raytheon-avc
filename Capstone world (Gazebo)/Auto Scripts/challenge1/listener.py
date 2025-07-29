import serial

port = '/dev/tty/USB0'
ser = serial.Serial(port=port, baudrate=57600, timeout=1)
while True:
    msg = ser.read(50)
    print(msg.decode('utf-8'))