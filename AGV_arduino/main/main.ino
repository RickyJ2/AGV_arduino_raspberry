#include <ArduinoJson.h>
#include <ArduinoJson.hpp>
#include "MotorDriver.h"
#include "Proximity.h"
#include "LimitSwitch.h"
#include "IMU.h"
#include "voltageReader.h"

MotorDriver motor(6,7,8,9,10,11);
Proximity container(5);
LimitSwitch uppperBumper(12);
LimitSwitch bellowBumper(13);
Kompas imu(4); 
VoltageReader powerbank(A2, 7.5, 8.3);
VoltageReader battery(A3, 7.2, 8.28);
//AGV State
float targetAngle = 90;
bool isDriving = false;
unsigned long previousMillis = 0;
unsigned long totalTime = 0;

void setup() {
  motor.init();
  motor.setLeftSpeed(200);
  motor.setRightSpeed(250);
  container.init();
  uppperBumper.init();
  bellowBumper.init();
  imu.init();
  powerbank.init();
  battery.init();
  Serial.begin(9600);
}

void loop() {
  JsonDocument info;
  info["type"] = "info";
  //Update all sensor data
  unsigned long currentMillis = millis();
  unsigned long currentSecond = currentMillis/1000;
  container.updateState(currentSecond);
  uppperBumper.updateState();
  bellowBumper.updateState();
  imu.updateState();
  powerbank.updateState();
  battery.updateState();
  //Send to raspberry
  JsonDocument data;
  data["type"] = "state";
  data["data"]["container"] = container.getState();
  data["data"]["collision"] = uppperBumper.getState() || bellowBumper.getState();
  vec3_t acceleration = imu.getAcceleration();
  float orientation = imu.getOrientation();
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

  if(Serial.available() > 0){
    //read via serial monitor
    // String cmd = Serial.readStringUntil('\n');
    JsonDocument input;
    deserializeJson(input, Serial);
    String type = input["type"];
    if(type == "direction"){
      int dir = input["direction"];
      targetAngle = dir;
      isDriving = true;
      previousMillis = currentMillis;
      totalTime = 0;
      info["data"] = "received direction";
      serializeJson(info, Serial);
      Serial.println();
    }else if(type == "cmd"){
      String cmd = input["cmd"];
      if(cmd == "stop"){
        motor.stop();
        isDriving = false;
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
      totalTime += currentMillis - previousMillis;
      info["data"] = "driving";
      serializeJson(info, Serial);
      Serial.println();
      if(totalTime >= 1400){
        motor.stop();
        isDriving = false;
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
  }else{
    motor.stop(); 
  }
}
