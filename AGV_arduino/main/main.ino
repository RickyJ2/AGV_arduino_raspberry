#include <ArduinoJson.h>
#include <ArduinoJson.hpp>
#include "MotorDriver.h"
#include "Proximity.h"
#include "LimitSwitch.h"
#include "IMU.h"
#include "voltageReader.h"
#include "PIDController.h"

MotorDriver motor(6,7,8,9,10,11);
Proximity container(5);
LimitSwitch uppperBumper(12);
LimitSwitch bellowBumper(13);
Kompas imu(4); 
VoltageReader powerbank(A2, 7.5, 8.3);
VoltageReader battery(A3, 7.2, 8.28);
PIDController pid(5,3,1);
//AGV State
float targetAngle = 90;
bool isDriving = false;

void setup() {
  motor.init();
  motor.setSpeed(240);
  container.init();
  uppperBumper.init();
  bellowBumper.init();
  imu.init();
  powerbank.init();
  battery.init();
  Serial.begin(9600);
}

void loop() {
  //Update all sensor data
  unsigned long currentSecond = millis()/1000;
  container.updateState(currentSecond);
  uppperBumper.updateState();
  bellowBumper.updateState();
  imu.updateState();
  powerbank.updateState();
  battery.updateState();
  //Send to raspberry
  JsonDocument data;
  data["container"] = container.getState();
  data["collision"] = uppperBumper.getState() || bellowBumper.getState();
  vec3_t acceleration = imu.getAcceleration();
  float orientation = imu.getOrientation();
  data["orientation"] = orientation;
  data["acceleration"]["x"] = acceleration.x;
  data["acceleration"]["y"] = acceleration.y;
  
   if(powerbank.getState() <= battery.getState()){
     data["power"] = powerbank.getState();
   }else{
     data["power"] = battery.getState();
   }
  serializeJson(data, Serial);
  Serial.println();

  if(Serial.available() > 0){
   JsonDocument input;
   deserializeJson(input, Serial);
   String type = input["type"];
   if(type == "direction"){
    int dir = input["direction"];
    targetAngle = dir;
   }else if(type == "cmd"){
    String cmd = input["cmd"];
    if(cmd == "stop"){
      motor.stop();
    }
   }
  }
  if(uppperBumper.getState() || bellowBumper.getState()){
    motor.stop();
  }else if(isDriving){
   int delta = orientation - targetAngle;
   if(targetAngle == 360) delta *= -1;
   if(abs(delta) < 3 || delta > 360 - 3){
    motor.forward();
    }else{
      if(targetAngle == 360 || targetAngle == 0){
        if(delta > 180){
          delta -= 360;
        }
      }
      if(delta < 0){
        motor.turnLeft();
      }else{
        motor.turnRight();
      }
    }
  }else{
    motor.stop(); 
  }
}
