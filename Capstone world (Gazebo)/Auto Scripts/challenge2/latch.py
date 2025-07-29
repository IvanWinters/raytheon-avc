import RPi.GPIO as GPIO
import time
 
GPIO.setmode(GPIO.BOARD)
GPIO.setup([8,10], GPIO.OUT)
 
GPIO.output(8, GPIO.LOW)
GPIO.output(10, GPIO.LOW)
 
def latch_open():
    GPIO.output(8, GPIO.LOW)
    GPIO.output(10, GPIO.HIGH)
    time.sleep(1)
    GPIO.output(8, GPIO.LOW)
    GPIO.output(10, GPIO.LOW)
 
    
def latch_close():
    GPIO.output(8, GPIO.HIGH)
    GPIO.output(10, GPIO.LOW)
    time.sleep(1)
    GPIO.output(8, GPIO.LOW)
    GPIO.output(10, GPIO.LOW)
 
open()
time.sleep(1)
close()
 
GPIO.cleanup()