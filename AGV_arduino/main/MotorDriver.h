#include "Motor.h"
#define maxSpeed 255
#define minSpeed 160
#define MOTOR_DELTAMAX 75
#define MOTOR_NOMINAL 180
/*
  Lebar Roda 36mm
  Jarak antar titik pusat Roda 189mm

  OUT1 OUT4 GND
  OUT2 OUT3 POS
*/
class MotorDriver{
  private:
    Motor left;
    Motor right;

  public:
    MotorDriver(int ENA, int IN1, int IN2, int IN3, int IN4, int ENB){
      this->left = Motor(ENB, IN3, IN4);
      this->right = Motor(ENA, IN1, IN2);
    }
    void init(){
      left.init();
      right.init();
      setSpeed(0);
    }
    void setSpeed(int speed){
      setLeftSpeed(speed);
      setRightSpeed(speed);
    }
    void setLeftSpeed(int speed){
      if(speed > maxSpeed){
        left.setSpeed(maxSpeed);
      }else if(speed < minSpeed){
        left.setSpeed(minSpeed);
      }else{
        left.setSpeed(speed);
      } 
    }
    void setRightSpeed(int speed){
      if(speed > maxSpeed){
        right.setSpeed(maxSpeed);
      }else if(speed < minSpeed){
        right.setSpeed(minSpeed);
      }else{
        right.setSpeed(speed);
      } 
    }
    //Manual Control
    void forward(){
      left.forward();
      right.forward();
    }
    void backward(){
      left.backward();
      right.backward();
    }
    void turnLeft(){
      left.backward();
      right.forward();
    }
    void turnRight(){
      left.forward();
      right.backward();
    }
    void stop(){
      left.stop();
      right.stop();
    }
    void movePID(int controlSteer){
      if( controlSteer>+MOTOR_DELTAMAX ) controlSteer= +MOTOR_DELTAMAX;
      if( controlSteer<-MOTOR_DELTAMAX ) controlSteer= -MOTOR_DELTAMAX;
      left.move(MOTOR_NOMINAL + controlSteer);
      right.move(MOTOR_NOMINAL - controlSteer);
    }
    int getLeftSpeed(){
      return left.getSpeed();
    }
    int getRightSpeed(){
      return right.getSpeed();
    }
};
