#include <ArduinoJson.h>
#include <ArduinoJson.hpp>
#include "MotorDriver.h"
#include "Proximity.h"
#include "voltageReader.h"

MotorDriver motor(6,7,8,9,10,11);
Proximity container(5);
VoltageReader powerbank(A2, 7.5, 8.3);
VoltageReader battery(A3, 7.2, 8.28);
//AGV State
unsigned long previousMainMillis = 0;

void setup() {
  motor.init();
//  motor.setLeftSpeed(230);
  motor.setLeftSpeed(200);
  motor.setRightSpeed(250);
  container.init();
  powerbank.init();
  battery.init();
  Serial.begin(9600);
}

void loop() {
  //Update all sensor data
  unsigned long currentMillis = millis();
  unsigned long currentSecond = currentMillis/1000;
  container.updateState(currentSecond);
  powerbank.updateState();
  battery.updateState();
  //Send to raspberry
  if( currentMillis - previousMainMillis > 500 ){
    JsonDocument data;
    data["type"] = "state";
    data["data"]["container"] = container.getState();
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
    Serial.println(type);
    // int input = cmd.toInt();
    // switch(input){
    //   case 1:{
    //     motor.forward();
    //     break;
    //   }
    //   case 2:{
    //     motor.turnLeft();
    //     break;
    //   }
    //   case 3:{
    //     motor.turnRight();
    //     break;
    //   }
    //   case 4:{
    //     motor.backward();
    //     break;
    //   }
    //   case 5:{
    //     motor.stop();
    //     break;
    //   }
    // }
  }
}
