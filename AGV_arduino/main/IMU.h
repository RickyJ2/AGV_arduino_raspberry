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
    Kompas(int int_pin = 2){
      interrupt_pin = int_pin;
    }

    void init(){
      // Untuk menginisialisasi Kompas
      Wire.begin();
//      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

      compass.initialize();
      pinMode(this->interrupt_pin, INPUT);

      //Verify connection
      testConnection();

      //Change with sensor's offset value
      //ID 01 [-2273,-2273] --> [0,17]  [-822,-821] --> [-20,3] [1105,1105] --> [16382,16389] [-243,-242] --> [-2,2]  [-211,-211] --> [0,1] [-49,-48] --> [0,2]
      compass.setXGyroOffset(-2273);
      compass.setYGyroOffset(-821);
      compass.setZGyroOffset(1105);
      compass.setXAccelOffset(-16389);
      compass.setYAccelOffset(-2);
      compass.setZAccelOffset(-49);
      //id 02 [697,698] --> [-11,8]  [-115,-114] --> [-12,5] [189,190] --> [16365,16385] [113,113] --> [0,1] [-35,-34] --> [-2,1]  [2,3] --> [0,4]
//      compass.setXGyroOffset(698);
//      compass.setYGyroOffset(-114);
//      compass.setZGyroOffset(189);
//      compass.setXAccelOffset(113);
//      compass.setYAccelOffset(-34);
//      compass.setZAccelOffset(2);
      compass.CalibrateGyro(6);
      devStatus = compass.dmpInitialize();
      if (devStatus == 0) {
        compass.CalibrateAccel(6);
        compass.CalibrateGyro(6);
        compass.PrintActiveOffsets();
        compass.setDMPEnabled(true);    
        attachInterrupt(digitalPinToInterrupt(this->interrupt_pin), dmpDataReady, RISING);
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
      //Konversi ke sistem +360 with respect to +x
      ypr[0] = (-1 * (int) (ypr[0] * 180 / M_PI) + 90 + 360) % 360; 
      // ypr[1] = (ypr[1] * 180 / M_PI); // PITCH
      // ypr[2] = (ypr[2] * 180 / M_PI); // ROLL
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
