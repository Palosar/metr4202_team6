#!/usr/bin/env python3
import pigpio
import time
if __name__ == "__main__":
    rpi = pigpio.pi()
    rpi.set_mode(18, pigpio.OUTPUT)
    rpi.set_servo_pulsewidth(18,2000)
    time.sleep(1)
    rpi.set_servo_pulsewidth(18,1250) 
    print("got here")
    #750 is the closed position
    #1250 is the grip box position
    #2000 is the open position
