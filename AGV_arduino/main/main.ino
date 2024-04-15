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
VoltageReader powerbank(A4, 7.5, 8.3); //A2 for id 01
VoltageReader battery(A3, 7.2, 8.28);
PIDController pid(2,5,1);
//AGV State
float targetAngle = 0;
bool isDriving = false;
bool direction = true; //true: forward, false: backward

void setup() {
  motor.init();
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
  data["orientation"] = imu.getOrientation();
  data["acceleration"]["x"] = acceleration.x;
  data["acceleration"]["y"] = acceleration.y;
  data["powerbank"] = powerbank.getPercent();
  data["baterai"] = battery.getPercent();
  
  // if(powerbank.getState() <= battery.getState()){
  //   data["power"] = powerbank.getState();
  // }else{
  //   data["power"] = battery.getState();
  // }
  serializeJson(data, Serial);
  Serial.println();

  if(Serial.available() > 0){
//    JsonDocument input;
//    deserializeJson(input, Serial);
//    String cmd = input["cmd"];
    //for control via Serial monitor
    String cmd = Serial.readStringUntil('\n');
    //Collission Routine
    if(cmd == "forward"){
      isDriving = true;
      direction = true;
    }else if(cmd == "backward"){
      isDriving = true;
      direction = false;
    }else if(cmd == "right"){
      targetAngle += 90;
      if (targetAngle > 180){
        targetAngle -= 360;
      }
      isDriving = false;
    }else if(cmd == "left"){
      targetAngle -= 90;
      if (targetAngle <= -180){
        targetAngle += 360;
      }
      isDriving = false;
    }else if(cmd == "stop"){
      isDriving = false;
    }
  }

  //PID Control for orientation
  double controlSignal = pid.compute(imu.getOrientation(), targetAngle);
  motor.setLeftSpeed(motor.getLeftSpeed() - controlSignal);
  motor.setRightSpeed(motor.getRightSpeed() + controlSignal);
  if(uppperBumper.getState() || bellowBumper.getState()){
    motor.stop();
  }else 
  if(isDriving){
    if(direction){
      motor.forward();
    }else{
      motor.backward();
    }
    
  }else{
    motor.stop();
  }
}
