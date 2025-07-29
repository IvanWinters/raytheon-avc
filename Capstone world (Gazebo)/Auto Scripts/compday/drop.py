from gpiozero import Motor
import time

PD_TIME = 0.576923121302

motor1 = Motor("BOARD8", "BOARD10")
motor2 = Motor("BOARD3", "BOARD5")

def open():
    motor1.forward()
    motor2.backward()
    time.sleep(PD_TIME)
    motor1.stop()
    motor2.stop()

def close():
    motor1.backward()
    motor2.forward()
    time.sleep(PD_TIME)
    motor1.stop()
    motor2.stop()

open()