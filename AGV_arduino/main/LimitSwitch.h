/*
  C - GND
  NO - not connected
  NC - digital Pin
*/

class LimitSwitch {
  private:
    int pin;
    bool state;
  
  public:
    LimitSwitch(int pinNumber) {
      pin = pinNumber;
      state = false;
    }

    void init(){
      pinMode(pin, INPUT_PULLUP);
      state = isPressed();
    }

    //update state after certain time
    void updateState(){
      state = isPressed();
    }

    bool getState(){
      return state;
    }
    
    bool isPressed() {
      return digitalRead(pin) == HIGH;
    }
};