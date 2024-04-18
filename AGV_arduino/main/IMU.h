#include <Wire.h>
#include <I2Cdev.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <vector_type.h>

volatile bool mpuInterrupt = false;

static void dmpDataReady() {
  mpuInterrupt = true;
}

class Kompas{
  private:
    MPU6050 compass;
    int interrupt_pin;
    
    bool dmpReady = false;  // set true if DMP init was successful
    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer
    uint8_t mpuIntStatus; 
    uint8_t devStatus; 

    Quaternion q;           // [w, x, y, z]         quaternion container
    VectorInt16 aa;         // [x, y, z]            accel sensor measurements
    VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
    VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
    VectorFloat gravity;    // [x, y, z]            gravity vector
    float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

  public:
    Kompas(int int_pin = 4){
      interrupt_pin = int_pin;
    }

    void init(){
      // Untuk menginisialisasi Kompas
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

      compass.initialize();
      pinMode(interrupt_pin, INPUT);

      //Verify connection
      testConnection();
      devStatus = compass.dmpInitialize();

      //Change with sensor's offset value
      //id 01 [-2275,-2274] --> [-12,5]  [-787,-786] --> [-6,12] [1091,1092] --> [16368,16386] [-233,-232] --> [-2,1]  [-211,-210] --> [0,2] [-53,-52] --> [0,3]
      compass.setXGyroOffset(-2274);
      compass.setYGyroOffset(-787);
      compass.setZGyroOffset(1091);
      compass.setXAccelOffset(-232);
      compass.setYAccelOffset(-211);
      compass.setZAccelOffset(-53);
      //id 02 [697,698] --> [-11,8]  [-115,-114] --> [-12,5] [189,190] --> [16365,16385] [113,113] --> [0,1] [-35,-34] --> [-2,1]  [2,3] --> [0,4]
//      compass.setXGyroOffset(698);
//      compass.setYGyroOffset(-114);
//      compass.setZGyroOffset(189);
//      compass.setXAccelOffset(113);
//      compass.setYAccelOffset(-34);
//      compass.setZAccelOffset(2);
      
      if (devStatus == 0) {
        compass.CalibrateAccel(6);
        compass.CalibrateGyro(6);
        compass.PrintActiveOffsets();
        compass.setDMPEnabled(true);    
        attachInterrupt(digitalPinToInterrupt(interrupt_pin), dmpDataReady, RISING);
        mpuIntStatus = compass.getIntStatus();
        dmpReady = true;
        // get expected DMP packet size for later comparison
        packetSize = compass.dmpGetFIFOPacketSize();
      }
    }

    bool testConnection(){
      return compass.testConnection();
    }

    void updateState(){
      compass.dmpGetCurrentFIFOPacket(fifoBuffer);
      compass.dmpGetQuaternion(&q, fifoBuffer);
      compass.dmpGetGravity(&gravity, &q);
      compass.dmpGetAccel(&aa, fifoBuffer);
      compass.dmpGetYawPitchRoll(ypr, &q, &gravity);
      //Update Orientation value +180 s/d -180
      ypr[0] = (ypr[0] * 180 / M_PI); 
      ypr[1] = (ypr[1] * 180 / M_PI); // PITCH
      ypr[2] = (ypr[2] * 180 / M_PI); // ROLL
      //Update Acceleration value
      compass.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    }

    float getOrientation(){
      return ypr[0]; //Yaw only
    }

    vec3_t getAcceleration(){
      return {aaReal.x, aaReal.y, aaReal.z};
    }
};
