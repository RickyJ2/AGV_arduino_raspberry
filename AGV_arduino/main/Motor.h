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
      this->speed = speed;
      analogWrite(enabPin, speed);
    }
    //Manual Control
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
    //Auto Move
    void move(int speed){
      if(speed == 0){
        stop();
        setSpeed(0);
      }else if(speed > 0){
        forward();
        setSpeed(speed);
      }else{
        backward();
        setSpeed(-speed);
      }
    }
    int getSpeed(){
      return speed;
    }
};