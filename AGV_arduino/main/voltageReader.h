#define VccMax 5
#define VccCorrection 1.017173 //VccCorrection = getVolt()/RealVoltage
#include "math.h"

/*
  GND_Battery
      |---<10K Ohm>- Analog Pin -<10K Ohm>- (+)Battery          
  GND_Arduino 
*/

class VoltageReader {
  private:
    int pin;
    float lowBat;
    float fullBat;
    int currentPercent;
    float currentVolt;
  
  public:
    // pinNumber: Analog Pin
    VoltageReader(int pinNumber, float lowBat, float fullBat) {
      pin = pinNumber;
      this-> lowBat = lowBat;
      this-> fullBat = fullBat;
      currentPercent = 100;
      currentVolt = 8.0;
    }

    void init(){
      updateState();
    }
    
    void updateState(){
      currentPercent = getPercent();
      currentVolt = getVolt();
    }

    int getState(){
      return currentPercent;
    }
    float getVoltState(){
      return currentVolt;
    }

    float getVolt(){
      return analogRead(pin) * VccMax / 1023.0 * 2 * VccCorrection;
    }

    int getPercent(){
      int percent = round((getVolt() - lowBat)/(fullBat - lowBat) * 100);
      if(percent < 0){
        return 0;
      }else if(percent > 100){
        return 100;
      }else{
        return percent;
      }
    }
};
