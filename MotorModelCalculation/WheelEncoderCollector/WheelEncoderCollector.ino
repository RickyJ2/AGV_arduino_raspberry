#define ENCODER_PIN 2
#define ENCODER_N   20
#define ENA 11
#define IN1 10
#define IN2 9
#define BATTERY_PIN A3
#define VccMax 5
#define VccCorrection 1.017173
 
unsigned long T1 = 0, T2 = 0, T;
bool MeasDone = 0;
int Motor_RPM = 0;
 
float getVolt(){
  return analogRead(BATTERY_PIN) * VccMax / 1023.0 * 2 * VccCorrection;
}

void INT0_ISR(void)
{
  if(MeasDone)
  {
    T2 = micros();
    T = T2 - T1;
    MeasDone = 0;
  }
  else
  {
    T1 = micros();
    MeasDone = 1;
  }
}
 
void setup()
{
  Serial.begin(9600);
  pinMode(ENCODER_PIN, INPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(BATTERY_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), INT0_ISR, RISING);
  while(!Serial){}
}
void loop()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  delay(10000);
  Serial.print(0);
  Serial.print(",");
  Serial.print(0);
  Serial.print(",");
  Serial.println(getVolt());
  delay(5000);
  for(int i = 60; i <= 255; i++){
    analogWrite(ENA, i);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    delay(250);
    int sum = 0;
    for(int j = 0; j < 20; j++){
      sum += (60000000) / (T * ENCODER_N);
      delay(250);
    }
    Serial.print(i);
    Serial.print(",");
    Serial.print(sum/20);
    Serial.print(",");
    Serial.println(getVolt());
  }
}