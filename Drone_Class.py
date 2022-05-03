from Motor_Classes import Motor
from machine import Pin, PWM
import time as time
import math
from ReadingIMU_Classes import IMU
class Drone:
    def __init__(self):
        self.max_power = 40
        self.min_power = 0
        self.mid_power = 0
        self.power_range = 7
    def initialize(self,M1,M2,M3,M4):
        
        for x in Motor.Motor_Array:
            x.PWM = 99
            x.duty(x.PWM)
        print('Calibrating High')    
        time.sleep(6)
        
        for x in Motor.Motor_Array:
            x.PWM = 0
            x.duty(x.PWM)
        print('Calibrating Low')    
        time.sleep(6)
        

    
    def takeoff(self,IMU,M1,M2,M3,M4):
        wait_time = .01
        dif = wait_time
        IMU.get_angles(dif)
        initial_roll = IMU.roll_angle
        initial_pitch = IMU.pitch_angle
        print(IMU.roll_angle,'---',IMU.pitch_angle)
        print('Drone Preparing Takeoff Procedure in')
        time.sleep(wait_time)
        for x in reversed(range(6)):
            print(x)
            time.sleep(1)
        t0 = time.ticks_ms()
        for i in range(self.max_power+2):
            time.sleep(.005)
            t1 = time.ticks_ms()
            dif = (t1-t0)/1000
            IMU.get_angles(dif)
            t0 = time.ticks_ms()
#             print(IMU.roll_angle,'---',IMU.pitch_angle)

            condition = self.emergency_power_down_user()
#             print(condition)
            if condition == False:
                while True:
                    M1.duty(0)
                    M2.duty(0)
                    M3.duty(0)
                    M4.duty(0)
            
            for x in Motor.Motor_Array:
                x.PWM = i
                x.duty(x.PWM)
                print(x.PWM)
#             if math.fabs(initial_roll-IMU.roll_angle) > 5 or math.fabs(initial_pitch-IMU.pitch_angle) > 5:
#                 print('Exceeded max angles')
#                 self.mid_power = i
#                 self.max_power = self.mid_power+self.power_range
#                 self.min_power = self.mid_power-self.power_range
#                 print(self.mid_power,'---',self.max_power,'---',self.min_power)
#                 break
            
            if i == self.max_power:
                self.mid_power = i
                self.max_power = self.mid_power+self.power_range
                self.min_power = self.mid_power-self.power_range
                print(self.mid_power,'---',self.max_power,'---',self.min_power)
                break
#                 self.emergency_power_down()
#                 print('Drone is fucked somehow')
#                 print('Please power down flight controller 1st and then disconnect drone battery')
#                 while True:
#                     M1.duty(0)
#                     M2.duty(0)
#                     M3.duty(0)
#                     M4.duty(0)
        
        
        
    def hover(self,IMU,PID,M1,M2,M3,M4):
        t0 = time.ticks_ms()
        e_tot = 0
        ki = PID.ki
        e_last = 0
        condition = True
#         file = open("data.csv","w")
#         file.write("M1"+","+"M2"+","+"M3"+","+"M4"+","+"Roll Angle"+","+"Pitch Angle"+","+"Time"+"\n")
        while condition == True:
            t1 = time.ticks_ms()
            dif = (t1-t0)/1000
#             time.sleep(.01)
            IMU.get_angles(dif)
            t0 = time.ticks_ms()
            PWM_dem,ki = PID.PID_theta(IMU,ki)
            PID.PID_Interpret(M1,M2,M3,M4,IMU,PWM_dem,self)
            print(IMU.roll_angle)
#             file.write("{}".format(M1.PWM)+","+"{}".format(M2.PWM)+","+"{}".format(M3.PWM)+","+"{}".format(M4.PWM)+","+"{0:0.4f}".format(IMU.roll_angle)+","+"{0:0.4f}".format(IMU.pitch_angle)+","+"{}".format(dif)+"\n")
            condition = self.emergency_power_down_user()
#         file.close()
        print('Powered Down')
        while True:
            M1.duty(0)
            M2.duty(0)
            M3.duty(0)
            M4.duty(0)
            time.sleep(1)
    
    def roll(self,IMU,PID,M1,M2,M3,M4):
        t0 = time.ticks_ms()
        e_tot = 0
        ki = PID.ki
        e_last = 0
        condition = True

        while condition == True:
            t1 = time.ticks_ms()
            dif = (t1-t0)/1000

            IMU.get_angles(dif)
            t0 = time.ticks_ms()
            
            PWM_dem,ki = PID.PID_theta(IMU,ki)
            PID.PID_Interpret(M1,M2,M3,M4,IMU,PWM_dem,self)
            
            condition = self.emergency_power_down_user()
            # condition = camera_control() # if the camera algorithm says it needs to roll, it will stop its pitch command and go to the other commands
        
        print('Powered Down')
        while True:
            M1.duty(0)
            M2.duty(0)
            M3.duty(0)
            M4.duty(0)
            time.sleep(1)
            
    def pitch(self,IMU,PID,M1,M2,M3,M4):
        t0 = time.ticks_ms()
        e_tot = 0
        ki = PID.ki
        e_last = 0
        condition = True

        while condition == True:
            t1 = time.ticks_ms()
            dif = (t1-t0)/1000

            IMU.get_angles(dif)
            t0 = time.ticks_ms()
            
            PWM_dem,ki = PID.PID_theta(IMU,ki)
            PID.PID_Interpret(M1,M2,M3,M4,IMU,PWM_dem,self)
    
            condition = self.emergency_power_down_user()
            # condition = camera_control() # if the camera algorithm says it needs to roll, it will stop its pitch command and go to the other commands
            
        print('Powered Down')
        while True:
            M1.duty(0)
            M2.duty(0)
            M3.duty(0)
            M4.duty(0)
            time.sleep(1)
                    
    def stabilize(self,M1,M2,M3,M4,IMU):
        pass
    
    
    
    
    @staticmethod
    def radio_initialize():
        led = Pin(25, Pin.OUT)                # LED
        csn = Pin(15, mode=Pin.OUT, value=1)  # chip select not
        ce  = Pin(14, mode=Pin.OUT, value=0)  # chip enable
        pipes = (b"\xe1\xf0\xf0\xf0\xf0", b"\xd2\xf0\xf0\xf0\xf0")
        
        spi = machine.SPI(0, baudrate=992063, polarity=0, phase=0, bits=8, sck=Pin(2), mosi=Pin(3), miso=Pin(4))
        nrf = NRF24L01(SPI(0), csn, ce, payload_size=4)
        
        nrf.open_tx_pipe(pipes[0]) # tx is transfer
        nrf.open_rx_pipe(1, pipes[1])# rx is recieve
        nrf.start_listening()
        led.value(0)
        
        self.auto_ack(nrf)
        return nrf
    
    @staticmethod
    def emergency_power_down_user():
        btn = Pin(28, Pin.IN, Pin.PULL_DOWN)
        led = Pin(25, Pin.OUT)  
        state = 0
        if state != btn.value():
            state = btn.value()
            led.value(state)
#         if nrf.any():
#             buf = nrf.recv()
#             got = struct.unpack("i", buf)[0]
#             led.value(got)
            while (Motor.Motor_Array[0].PWM != 0 or Motor.Motor_Array[1].PWM != 0 or Motor.Motor_Array[2].PWM != 0 or Motor.Motor_Array[3].PWM != 0):
                time.sleep(.025)
                for x in Motor.Motor_Array:
                    x.PWM = x.PWM-1
                    x.duty(x.PWM)
                    if x.PWM <= 0:
                        x.PWM = 0
                    print(x.PWM)
            return False
        else:
            return True
                
    @staticmethod
    def emergency_power_down():
        while (Motor.Motor_Array[0].PWM != 0 or Motor.Motor_Array[1].PWM != 0 or Motor.Motor_Array[2].PWM != 0 or Motor.Motor_Array[3].PWM != 0):
            time.sleep(.025)
            for x in Motor.Motor_Array:
                x.PWM = x.PWM-1
                x.duty(x.PWM)
                if x.PWM <= 0:
                    x.PWM = 0
#                 print(x.PWM)
        
    
    @staticmethod
    def auto_ack(nrf):
        nrf.reg_write(0x01, 0b11111000)  # enable auto-ack on all pipes
        
class PID(Drone):
    def __init__(self,SPr,SPp,SPz,roll_max,pitch_max,z_max,kp,ki,kd,drone):
        self.SPr = SPr
        self.SPp = SPp
        self.SPz = SPz
        
        if self.SPr<0:
            self.roll_max = self.SPr-10

        else:
            self.roll_max = self.SPr+10
            
        self.pitch_max = pitch_max
        self.z_max = z_max
        
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.e_last = 0
        self.e_tot = 0
        self.threshold = .02
        self.PWM_max = 1
        self.PWM_min = -self.PWM_max
        self.max_power = drone.max_power
        self.min_power = drone.min_power
        self.mid_power = drone.mid_power
        self.power_range = drone.power_range
        
        
        
    
    def PID_theta(self,IMU,ki): # e_tot = 0 for first run
        e = IMU.roll_angle-self.SPr#+math.fabs((IMU.pitch_angle-self.SPp))
        theta_max = math.fabs((self.roll_max-self.SPr))#+math.fabs((self.pitch_max-self.SPp))
        e_scaled = (e/theta_max)
        
        
        self.e_tot += e_scaled
        delt = IMU.sampling
        
        P_term = self.kp*e_scaled
        I_term = ki*self.e_tot*delt
        D_term = float(self.kd*((e_scaled-self.e_last)/delt))
#         print(I_term)
        PWM_dem = P_term+I_term+D_term
        #print(roll_angle,'---',pitch_angle, '---',e_tot,'---',P_term)
        if PWM_dem > self.PWM_max: # Stops integral windup
            ki = 0
            PWM_dem = self.PWM_max
        if PWM_dem<self.PWM_min:
            ki = 0
            PWM_dem = self.PWM_min
        
        
        else:
            ki = self.ki
        self.e_last = e_scaled
        
        return PWM_dem,ki
    
    def PID_Interpret(self,M1,M2,M3,M4,IMU,PWM_dem,drone):
          # Safety shutdown - disable if tuning on test stand
          
#         if math.fabs(IMU.roll_angle)>60 or math.fabs(IMU.pitch_angle)>60:
#             drone.emergency_power_down()

#             while True:
#                 M1.duty(0)
#                 M2.duty(0)
#                 M3.duty(0)
#                 M4.duty(0)
#                 time.sleep(1)
        
        if PWM_dem>0:
            M1.PWM = self.mapped(PWM_dem)
            M3.PWM = self.mid_power-(M1.PWM-self.mid_power)
            
            M2.PWM = M3.PWM
            M4.PWM = M1.PWM

            M1.duty(M1.PWM)
            M2.duty(M2.PWM)
            M3.duty(M3.PWM)
            M4.duty(M4.PWM)
            
        elif PWM_dem<0:
            M1.PWM = self.mapped(PWM_dem)
            M3.PWM = self.mid_power-(M1.PWM-self.mid_power)
            
            M2.PWM = M3.PWM
            M4.PWM = M1.PWM

            M1.duty(M1.PWM)
            M2.duty(M2.PWM)
            M3.duty(M3.PWM)
            M4.duty(M4.PWM)
                
             
#         elif IMU.roll_angle>threshold_roll_up or IMU.pitch_angle>threshold_pitch_up:
#             if IMU.pitch_angle<threshold_pitch_down:
#                 M4_PWMdif = int(PWM_dem*100)-M4.PWM
#                     
#                 M4.PWM = int((M4_PWMdif)+M4.PWM)
#                 M2.PWM = int(M2.PWM-(M4_PWMdif))
#                 if M2.PWM+M4.PWM != 98:
#                     M2.PWM = int((98-(M2.PWM+M4.PWM))+M2.PWM)
#                 M4.duty(M4.PWM)
#                 M2.duty(M2.PWM)
#                 print('M4 PWM: ', M4.PWM,'---','M2 PWM: ', M2.PWM)
#                     
#             elif IMU.roll_angle<threshold_roll_down:
#                 #print(M2.PWM)
#                 M2_PWMdif = int((PWM_dem*100))-M2.PWM
#                     
#                 M2.PWM = int((M2_PWMdif)+M2.PWM)
#                 M4.PWM = int(M4.PWM-(M2_PWMdif))
#                 if M2.PWM+M4.PWM != 98:
#                     M4.PWM = int((98-(M2.PWM+M4.PWM))+M4.PWM)
#                 M4.duty(M4.PWM)
#                 M2.duty(M2.PWM)
#                 print('M4 PWM: ', M4.PWM,'---','M2 PWM: ', M2.PWM)
#             elif IMU.pitch_angle>threshold_pitch_up and IMU.roll_angle>threshold_roll_up:
#                 M1_PWMdif = int(PWM_dem*100)-M1.PWM
#                 M1_PWM = int((M1_PWMdif)+M1.PWM)
#                 M3_PWM = int(M3.PWM-(M1_PWMdif))
#                 if M1.PWM+M3.PWM != 98:
#                     M3.PWM = int((98-(M3.PWM+M1.PWM))+M3.PWM)
#                 print('M1 PWM: ',M1.PWM,'---','M3 PWM: ', M3.PWM)
#                 M1.duty(M1.PWM)
#                 M3.duty(M3.PWM)
#         elif IMU.roll_angle<threshold_roll_down and IMU.pitch_angle<threshold_pitch_down:
#             M3_PWMdif = int(PWM_dem*100)-M3.PWM
#             M3.PWM = int((M3_PWMdif)+M3.PWM)
#             M1.PWM = int(M1.PWM-(M3_PWMdif))
#             if M1.PWM+M3.PWM != 98:
#                 M1.PWM = int((98-(M3.PWM+M1.PWM))+M1.PWM)
#             print('M1 PWM: ',M1.PWM,'---','M3 PWM: ', M3.PWM)
#             M1.duty(M1.PWM)
#             M3.duty(M3.PWM)
        
        
        
        
    def mapped(self,PWM_dem):
        value = int(((PWM_dem-self.PWM_min)*(self.max_power-self.min_power)/(self.PWM_max-self.PWM_min))+self.min_power)

        return value
    
    @staticmethod
    def dt(sampling_rate):
        t0 = time.ticks_ms()
        time.sleep(sampling_rate)
        t1 = time.ticks_ms()
    
        dt = (t1-t0)/1000
        return dt
        