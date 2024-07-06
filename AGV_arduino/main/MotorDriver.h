#include "Motor.h"
/*
  OUT1 OUT4 | GND
  OUT2 OUT3 | POS
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
      left.setSpeed(speed);
      right.setSpeed(speed);
    }
    void move(int leftPWM, int rightPWM){
      left.move(leftPWM);
      right.move(rightPWM);
    }
    void stop(){
      left.stop();
      right.stop();
    }
    int getLeftSpeed(){
      return left.getSpeed();
    }
    int getRightSpeed(){
      return right.getSpeed();
    }
};
