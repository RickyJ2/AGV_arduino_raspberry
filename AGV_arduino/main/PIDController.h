class PIDController{
  private:
    unsigned long previousMillis = 0;
    double errSum, lastErr;
    double kp, ki, kd;

  public:
    PIDController(double kp, double ki, double kd){
      this->kp = kp;
      this->ki = ki;
      this->kd = kd;
      this->errSum = 0;
      this->lastErr = 0;
    }

    double compute(double Input, double SetPoint){
      unsigned long currentMillis = millis();
      double deltaT = (double) (currentMillis - previousMillis);

      double error = SetPoint - Input;
      errSum += (error * deltaT);
      double dErr = (error - lastErr) / deltaT;

      double Output = kp * error + ki * errSum + kd * dErr;

      lastErr = error;
      previousMillis = currentMillis;
      return Output;
    }
};