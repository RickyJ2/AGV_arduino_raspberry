#include <ArduinoJson.h>
#include <ArduinoJson.hpp>
#include "MotorDriver.h"
#include "Proximity.h"
#include "IMU.h"
#include "voltageReader.h"

#define IDLE 0 
#define AUTO 1
#define MANUAL 2

#define STOP 0
#define FORWARD 1
#define LEFT 2
#define RIGHT 3
#define BACKWARD 4

MotorDriver motor(6,7,8,9,10,11);
Proximity container(5);
Kompas imu(2);
VoltageReader powerbank(A2, 7.5, 8.3);
VoltageReader battery(A3, 7.2, 8.28);
//AGV State
int typeMoving = IDLE;
unsigned long previousMainMillis = 0;
//MODE AUTO
float targetAngle = 90;
unsigned long previousMillis = 0;
unsigned long totalTime = 0;
unsigned int duration = 0;
//MODE MANUAL
int typeMove = STOP;

void setup() {
  motor.init();
//  motor.setLeftSpeed(230);
  motor.setLeftSpeed(200);
  motor.setRightSpeed(250);
  container.init();
  imu.init();
  powerbank.init();
  battery.init();
  Serial.begin(9600);
}

void loop() {
  //Update all sensor data
  unsigned long currentMillis = millis();
  container.updateState(currentMillis/1000);
  imu.updateState();
  powerbank.updateState();
  battery.updateState();
  vec3_t acceleration = imu.getAcceleration();
  float orientation = imu.getOrientation();
  //Send to raspberry
  if( currentMillis - previousMainMillis > 500 ){
    JsonDocument data;
    data["type"] = "state";
    data["data"]["container"] = container.getState();
    data["data"]["orientation"] = orientation;
    data["data"]["acceleration"]["x"] = acceleration.x;
    data["data"]["acceleration"]["y"] = acceleration.y;
    if(powerbank.getState() <= battery.getState()){
      data["data"]["power"] = powerbank.getState();
    }else{
      data["data"]["power"] = battery.getState();
    }
    serializeJson(data, Serial);
    Serial.println();
    previousMainMillis = currentMillis;
  }
  
  if(Serial.available() > 0){
    String cmd = Serial.readStringUntil('\n');
    JsonDocument input;
    deserializeJson(input, cmd);
    String type = input["type"];
    if(type == "direction"){
      targetAngle = input["direction"];
      duration = input["duration"];
      typeMoving = AUTO;
      previousMillis = currentMillis;
      totalTime = 0;
    }else if(type == "move"){
      typeMove = input["data"];
      typeMoving = MANUAL;
    }else if(type == "control"){
      float leftVolt = input["left"];
      float rigthVolt = input["volt"];
      if (leftVolt == 0){
        motor.left.stop();
      }else{
        int leftPWM = leftVolt * 255 / battery.getState();
        motor.setLeftSpeed(leftPWM);
        if(leftPWM < 0){
          motor.left.backward();
        }else{
          motor.left.forward();
        }
      }
      if(rigthVolt == 0){
        motor.right.stop();
      }else{
        int rightPWM = leftVolt * 255 / battery.getState(); 
        motor.setRightSpeed(rightPWM);
      
        if(rightPWM < 0){
          motor.right.backward();
        }else{
          motor.right.forward();
        }  
      }
    }
  }

  // switch(typeMove){
  //   case IDLE:{
  //     motor.stop();
  //     break;
  //   }
  //   case MANUAL:{
  //     controlMovement(typeMove);
  //     break;
  //   }
  //   case AUTO:{
  //     driving(orientation, currentMillis);
  //     break;
  //   }
  // }
}

void driving(float orientation, unsigned long currentMillis){
  int delta = orientation - targetAngle;
  if(targetAngle == 360) delta *= -1;
  if(abs(delta) < 3 || delta > 360 - 3){
    motor.forward();
    totalTime += currentMillis - previousMillis;
    if(totalTime >= duration){
      motor.stop();
      typeMove = IDLE;
      JsonDocument notif;
      notif["type"] = "notif";
      notif["data"] = totalTime;
      serializeJson(notif, Serial);
      Serial.println();
    }
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
  previousMillis = currentMillis;
}

void controlMovement(int dir){
  switch(dir){
    case FORWARD:{
      motor.forward();
      break;
    }
    case LEFT:{
      motor.turnLeft();
      break;
    }
    case RIGHT:{
      motor.turnRight();
      break;
    }
    case BACKWARD:{
      motor.backward();
      break;
    }
    case STOP:{
      motor.stop();
      break;
    }
    default:{
      motor.stop();
      break;
    }
  }
}
