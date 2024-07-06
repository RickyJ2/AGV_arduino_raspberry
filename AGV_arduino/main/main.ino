#include "Header.h"

MotorDriver motor(6,7,8,9,10,11);
Proximity container(5);
VoltageReader batteryRaspberry(A2, 7.5, 8.3);
VoltageReader batteryMotor(A3, 7.2, 8.0);

//AGV State
unsigned long previousMainMillis = 0;

void setup() {
  motor.init();
  container.init();
  batteryRaspberry.init();
  batteryMotor.init();
  Serial.begin(9600);
}

void loop() {
  //Update all sensor data
  unsigned long currentMillis = millis();
  container.updateState(currentMillis/1000);
  batteryRaspberry.updateState();
  batteryMotor.updateState();
  //Send to raspberry
  if(currentMillis - previousMainMillis > 500){
    JsonDocument data;
    data["type"] = "state";
    data["data"]["container"] = container.getState();
    if(batteryRaspberry.getState() <= batteryMotor.getState()){
      data["data"]["power"] = batteryRaspberry.getState();
    }else{
      data["data"]["power"] = batteryMotor.getState();
    }
    data["data"]["power"] = batteryMotor.getState();
    serializeJson(data, Serial);
    Serial.println();
    previousMainMillis = currentMillis;
  }
  //Receive command from raspberry
  if(Serial.available() > 0){
    String cmd = Serial.readStringUntil('\n');
    JsonDocument input;
    deserializeJson(input, cmd);
    float leftVolt = input["left"];
    float rigthVolt = input["right"];
    int leftPWM = leftVolt * 255 / batteryMotor.getVoltState();
    int rightPWM = rigthVolt * 255 / batteryMotor.getVoltState();
    motor.move(leftPWM, rightPWM);
  }
}
