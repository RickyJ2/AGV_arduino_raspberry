//Don't forget configurate potentiometer on Sensor
class Proximity {
  private:
    int pin;
    bool state;
    unsigned long previousSecond = 0;
  
  public:
    // Constructor
    Proximity(int pinNumber) {
      pin = pinNumber;
      state = false;
    }

    void init(){
      pinMode(pin, INPUT);
      state = isObjectDetected();
    }
    //update state after certain time
    void updateState(unsigned long current){
      if(current - previousSecond >= 3){
        previousSecond = current;
        state = isObjectDetected();
      }
    }
    
    bool getState(){
      return state;
    }

    bool isObjectDetected() {
      return digitalRead(pin) == LOW;
    }
};