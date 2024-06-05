#define maxSpeed 255
#define minSpeed 160

class Motor {
  private:
    int enabPin;
    int in1;
    int in2;
    int speed;

  public:
    Motor(){}
    Motor(int enabPin, int in1, int in2){
      this->enabPin = enabPin;
      this->in1 = in1;
      this->in2 = in2;
    }
    void init(){
      pinMode(enabPin, OUTPUT);
      pinMode(in1, OUTPUT);
      pinMode(in2, OUTPUT);
    }
    void setSpeed(int speed){
      if(speed < 0){
        speed = -1 * speed;
      }
      if(speed > maxSpeed){
        speed = maxSpeed;
      }
      this->speed = speed;
      analogWrite(enabPin, speed);
    }
    void move(int pwm){
      setSpeed(pwm);
      if(pwm == 0){
        stop();
      }else if(pwm < 0){
        backward();
      }else{
        forward();
      }
    }
    void forward(){
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
    }
    void backward(){
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
    }
    void stop(){
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
    }
    int getSpeed(){
      return speed;
    }
};