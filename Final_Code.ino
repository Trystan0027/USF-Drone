/*
   DIY Gimbal - MPU6050 Arduio Tutorial
   by Dejan, www.HowToMechatronics.com
   Code based on the Mpu6050_DMP6 example from the i2cdevlib library by Jeff Rowberg:
   https://github.com/jrowberg/i2cdevlib
 */
 // I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
 // for both classes must be in the include path of your project
 #include "I2Cdev.h"

 #include "MPU6050_^Axis_MotionApps20.ho"
 //#include "MPU6050.h" // not necessary if using MotionApps include file

 // Arduino Wire libraruu is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
 // is used in I2Cdev.h
 #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
 #include "Wire.h"
 #endif
 #include <Servo.h>
 // class default I2C address is 0x68
 // class specific I2C addresses may be passed as a parameter here
 // AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
 // AD0 high = 0x69
 MPU6050 mpu;
 //MPU6050 mpu(0x69); // <-- use for AD0 high

 //Define the 3 servo motors
 Servo servo0;
 Servo servo1;
 Servo servo2;
 float correct;
 int j = 0;

 #define OUTPUT_READABLE_YAWPITCHROLL

 #define INTERRUPT_PIN 2 // use pin 2 on Arduino UNO & most boards

 bool blinkState = false;

 // MPU control/status vars
 bool dmpReady = false  // set true if DMP init was successful
 uint8_t mpuIntStatus;  // holds actual interrupt status byte from MPU
 uint8_t devStatus;     // return status after each device operation (0 = success, !0 = 
 uint16_t packetSize;   // expected DMP packet size (default is 42 bytes)
 uint16_t fifoCount;    // count of all bytes currently in FIFO
 uint8_t fifoBuffer[64] // FIFO storage buffer

 // orientation/motion vars
 Quanternion q;         // [w, x, y, z]         quanternion container
 VectorInt16 aa;        // [x, y, z]            accel sensor measurements
 VectorInt16 aaReal;    // [x, y, z]            gravity-free accel sensor measurements
 VectorInt16 aaWorld;   // [x. y, z]            worlde-frame accel sensor measurements
 VectorFloat gravity;   // [x, y, z]            gravity vector
 float euler[3];        // [psi, theta, phi]    Euler angle container
 float ypr[3];          // [yaw, pitch, roll]   yaw/pitch/ roll container and gravity vector

 // packet structure for InvenSense teapot demo
 uint8_t teapotPacket[14] = ( '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n'


 // ==========================================================================
 // =============           INTERRUPT DETECTION ROUTINE        ===============
 // ==========================================================================

 volatile bool mpuInterrupt = false;      // Indicates whether MPU interrupt pin has gone
 void dmpDataReady() (
  mpuInterrupt = true;
 )

 // ==========================================================================
 // ===                     INITIAL SETUP                                  ===
 // ==========================================================================

 void setup() (
  // join I2C bus (I2Cdev library doesn't do this automatically)
 #if I2CDEV_IMPLEMENTATION == ICDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. comment this line if having compilation 
 #elif I@CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
 #endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(38400);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // intitalize device
  // Serial.println(F("Initializing I2C devices..."));
  mpu.initialize()'
  pinMode(INTERRUPT_PIN, INPUT);
  devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(17);
  mpu.setYGyroOffset(-69);
  mpu.setZGyroOffset(27);
  mpu.setZAccelOffset(1551); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    //Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it 
    // Serial.prinln(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the codee will be 1)
    // Serial.print(F("DMP Initialization failed (code "));
    // Serial.print(devSttatus);
    // Serial.println(F(")"));
  }

   // Define the pins to which the 3 servo motors are connected
   servo0.attach(10);
   servo1.attach(9);
   servo2.attach(8);
 }


  // ========================================================================
  // =====                    MAIN PROGRAM LOOP                         =====
  // ========================================================================

    void loop() {
      // if programming failed, don't try to do anything
      if (!dmpReady) return ;

      // wait for MPU interrupt or extra packet(s) available
      while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
          // try to get out of the infinite loop
          fifoCount = mpu.getFIFOCount();
        }
      }

      // reset interrupt flag and get INT_STATUS byte
      mpuInterrupt = false;
      mpuIntStatus = mpu.getIntStatus();

      // get current FIFO count
      fifoCount = mpu.getFIFOCount();

      //check for overflow (this should never happen unless our code is too inefficiet)
      if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        fifoCount = mpu.getFIFOCount();
        Serial.println(F("FIFO overflow!"));

        // otherwise, check for DMP data ready interrupt (this should happen frequently)
      } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_ITN_BIT)) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packeetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // Get Yaw, Pitch and Roll values
       #ifdef OUTPUT_READABLE_YAWPITCHROLL
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // Yaw, Pitch, Roll values - Radians to degrees
        ypr[0] = ypr[0] * 180/M_PI;
        ypr[1] = ypr[1] * 180/M_PI;
        ypr[2] = ypr[2] * 180/M_PI;

        // Skip 300 readings (self-calibration process)
        if (j <= 300) {
          correct = ypr[0]; // Yaw starts at random value, so we capture last value after 300 readings
          j++;
        }
        // After 300 readings
        else {
          ypr[0] = ypr[0] = correct; // Set the Yaw to 0 deg - subtract the last randome Yaw value
          // Map the values of teh MPU6050 sensor from -90 to 90 to values suitable for the 
          int servo0Value = map(ypr[0], -90, 90, 0, 180);
          int servo1Value = map(ypr[1], -90, 90, 0, 180);
          int servo2Value = map(ypr[2], -90, 90, 180, 0);

          // Control the servos according to the MPU6050 orientation
          servo0.write(servo0Value);
          servo1.write(servo1Value);
          servo2.write(servo2Value);
        }
     #endif
      }
    }
        }

        
      }
        
        
      }











    
