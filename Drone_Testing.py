from Drone_Class import Drone, PID
from Motor_Classes import Motor
from machine import Pin, PWM
from ReadingIMU_Classes import IMU
import time as time
import math

IMU1 = IMU(12,13,400000,0.01) # (sda pin,scl pin,I2C frequency, sampling rate (in seconds), accelerometer weight (how much accel reading you want<.5), gyro y noise threshold, gyro x noise threshold)
drone = Drone()
M1 = Motor(Pin(9),1015,2001)
M2 = Motor(Pin(6),1251,1958)
M3 = Motor(Pin(8),1017,1995)
M4 = Motor(Pin(7),1056,1955)
# Test Motor Throttle up
drone.initialize(M1,M2,M3,M4)
IMU1.calibration()
drone.takeoff(IMU1,M1,M2,M3,M4)
PID_Hover = PID(0,0,0,10,10,30,.105,0.035,0.0001,drone) #.12 is the move for p
PID_Roll = PID(20,0,0,30,10,30,.107,0.03,0.0001,drone)
# drone.roll(IMU1,PID_Roll,M1,M2,M3,M4)
drone.hover(IMU1,PID_Hover,M1,M2,M3,M4)
# drone.roll(IMU1,PID_Roll,M1,M2,M3,M4)
# M1 CW
# M2 CCW
