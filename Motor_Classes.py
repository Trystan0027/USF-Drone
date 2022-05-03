from machine import Pin, PWM
import time as time
import math
from ReadingIMU_Classes import IMU

class Motor(PWM):
    Motor_Array = []
    
    def __init__(self,pin:Pin,min_throttle,max_throttle):
        super().__init__(pin)
        self.min_throttle = min_throttle
        self.max_throttle = max_throttle
        self.frequency = int(1000000/self.max_throttle)
        self.PWM_bottom = self.min_throttle/(1000000/self.frequency)
        self.PWM_mid = ((1-self.PWM_bottom)/2)+self.PWM_bottom
        self.freq(self.frequency)
        self.PWM = 0
        Motor.Motor_Array.append(self)
        
    def duty(self,duty): # duty is an integer between 0 and 100
        conversion = duty/100
        duty_cycle = int((65535-65535*self.PWM_bottom)*conversion+65535*self.PWM_bottom)
        #print(conversion, '------------', duty_cycle)
        self.duty_u16(duty_cycle)
    
        

# M1 = Motor(Pin(9),1015,2001)
# M2 = Motor(Pin(6),1015,1611)
# M3 = Motor(Pin(8),1112,2100)
# M4 = Motor(Pin(7),1000,2000)
# print(Motor.Motor_Array[0].PWM)
# for i in reversed(range(Motor.Motor_Array[0].PWM)):
#     Motor.Motor_Array[0].PWM = i
#     print(Motor.Motor_Array[0].PWM)
# # print(len(Motor.Motor_Array))
#IMU1 = IMU(12,13,400000,0.01,.75,.25,.25) # (sda pin,scl pin,I2C frequency, sampling rate (in seconds), accelerometer weight (how much accel reading you want<.5), gyro y noise threshold, gyro x noise threshold)
#gx_cal,gy_cal,gz_cal,roll_angle_gyro, pitch_angle_gyro = IMU1.gyro_calibration() # Calibrate the gyroscope and gives back initial pitch and roll readings



