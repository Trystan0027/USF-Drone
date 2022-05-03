from imu import MPU6050
import time
from machine import Pin, I2C
import array as ar
import time
import math

#----------------classes-------------------#
class IMU:
    calibration_time = 40 
    deadband = 200
    def __init__(self,sdaPin,sclPin,freq,sampling_rate):
        self.sdaPin = sdaPin
        self.sclPin = sclPin
        self.freq = freq
        self.sampling = sampling_rate
        self.accel_weight = .9
        self.gyro_weight = 1-self.accel_weight
        self.noise_gyro_y = .25
        self.noise_gyro_x = .25
        i2c = I2C(0,sda = Pin(self.sdaPin),scl = Pin(self.sclPin), freq = self.freq)
        self.imu = MPU6050(i2c)
        self.roll_angle_gyro = 0
        self.roll_angle = 0
        self.pitch_angle_gyro = 0
        self.pitch_angle = 0
        self.gx_cal = 0
        self.gy_cal = 0
        self.gz_cal = 0
        self.angle_change = 50
           
    def calibration(self):
        gx = ar.array('f') # create gyro x array
        gy = ar.array('f') # create gyro y array
        gz = ar.array('f') # create gyro z array
        dt_array = ar.array('f') # create a time array
        
        Loop_Var = True # initialize variable to get into the loop
        while Loop_Var == True:
            # Creating arrays 
            gx.append(round(self.imu.gyro.x)) # Appending the gyro x array
            gy.append(round(self.imu.gyro.y)) # Appending the gyro y array
            gz.append(round(self.imu.gyro.z)) # Appending the gyro z array
            delt = self.dt(self.sampling)
            dt_array.append(delt)
            
            if len(dt_array) >= self.calibration_time:
                Loop_Var = False
            else:
                Loop_Var = True
                
        self.gx_cal,self.gy_cal,self.gz_cal = self.Average(gx,gy,gz)
        roll_angle_initial = -math.degrees(math.atan((self.imu.accel.x)/(self.imu.accel.z)))
        pitch_angle_initial = -math.degrees(math.atan((self.imu.accel.y)/(self.imu.accel.z)))
        delt = self.dt(self.sampling)
        gyro_read_x = ((self.imu.gyro.x)-self.gx_cal)*delt
        gyro_read_y = ((self.imu.gyro.y)-self.gy_cal)*delt
        self.roll_angle_gyro = gyro_read_y*self.gyro_weight + roll_angle_initial*self.accel_weight
        self.pitch_angle_gyro = gyro_read_x*self.gyro_weight + pitch_angle_initial*self.accel_weight
    

    def get_angles(self,dif):
        z_accel = self.imu.accel.z
#         if z_accel == 0:
#             z_accel = .01
        roll_angle_array = []
        pitch_angle_accel =-math.degrees(math.atan((self.imu.accel.y)/(self.imu.accel.z)))
        for i in range(5):
            roll_angle_array.append(-math.degrees(math.atan((self.imu.accel.x)/(z_accel)))*self.accel_weight)
        a = 0
        for i in roll_angle_array:
            a = a+i
        roll_angle_accel = a/len(roll_angle_array)
        gyro_read_x = float((self.imu.gyro.x)-self.gx_cal)
        gyro_read_y = float((self.imu.gyro.y)-self.gy_cal)
        if (gyro_read_x< self.noise_gyro_x and gyro_read_x>(-self.noise_gyro_x)) or (gyro_read_x>self.deadband or gyro_read_x<-self.deadband): # deadbands
            gyro_read_x = 0
        if (gyro_read_y< self.noise_gyro_y and gyro_read_y>(-self.noise_gyro_y)) or (gyro_read_y>self.deadband or gyro_read_y<-self.deadband): # deadbands
            gyro_read_y = 0
#         print(gyro_read_y,'---','---',self.imu.gyro.y)
#         print(roll_angle_accel)
        gx_value = (gyro_read_x)*dif
        gy_value = (gyro_read_y)*dif
        self.pitch_angle_gyro -= gx_value
        self.roll_angle_gyro += gy_value
        #print(gy_value, '-------------', roll_angle_gyro)
        self.pitch_angle = self.pitch_angle_gyro*self.gyro_weight+pitch_angle_accel # Complementary filter
        roll_angle = self.roll_angle_gyro*self.gyro_weight+roll_angle_accel # Complementary filter
        
        if math.fabs(self.roll_angle-roll_angle)>self.angle_change:
            self.roll_angle = 0
        else:
            self.roll_angle = roll_angle
#         print(self.roll_angle)
        #print(roll_angle_accel)

    

    
    @staticmethod
    def Average(gx,gy,gz):
        tx = 0
        ty = 0
        tz = 0
        for x in gx:
            tx = tx + x
        for y in gy:
            ty = ty + y
        for z in gz:
            tz = tz + z
            gx_avg = tx/len(gx)
            gy_avg = ty/len(gy)
            gz_avg = tz/len(gz)
    
        return gx_avg, gy_avg, gz_avg
    
    @staticmethod
    def dt(sampling_rate):
        t0 = time.ticks_ms()
        time.sleep(sampling_rate)
        t1 = time.ticks_ms()
    
        dt = (t1-t0)/1000
        #print(dt)
        return dt
    
# IMU1 = IMU(12,13,400000,0.01) # (sda pin,scl pin,I2C frequency, sampling rate (in seconds), accelerometer weight (how much accel reading you want<.5), gyro y noise threshold, gyro x noise threshold)
# IMU1.calibration() # Calibrate the gyroscopes
# while True:
#     IMU1.get_angles(.01)
#     time.sleep(.01)
#     print(IMU1.roll_angle, '------------', IMU1.pitch_angle)




        



