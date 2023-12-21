int EnRpin=7;
int MotorRpin1 = 50;
int MotorRpin2 = 48;

int EnLpin=6;
int MotorLpin1 = 46;
int MotorLpin2 = 44;

void setup() {
  // put your setup code here, to run once:
  pinMode(EnRpin, OUTPUT);
  pinMode(EnLpin, OUTPUT);
  pinMode(MotorRpin1, OUTPUT);
  pinMode(MotorRpin2, OUTPUT);
  pinMode(MotorLpin1, OUTPUT);
  pinMode(MotorLpin2, OUTPUT);
  analogWrite(EnRpin, 220);
  analogWrite(EnLpin, 220);
}
// IF L1,R1 pin high and L2,R2 low -->Motor Forward
// IF L1,R1 pin low and L2,R2 high -->Motor Reverse
void loop() {
  // put your main code here, to run repeatedly:  

  digitalWrite(MotorRpin1, HIGH);
  digitalWrite(MotorRpin2, LOW);
  delay(1000);
  digitalWrite(MotorRpin1, LOW);
  digitalWrite(MotorRpin2, LOW);
  delay(1000);
  digitalWrite(MotorRpin1, LOW);
  digitalWrite(MotorRpin2, HIGH);
  delay(1000);
  digitalWrite(MotorRpin1, LOW);
  digitalWrite(MotorRpin2, LOW);
  delay(1000);

  digitalWrite(MotorLpin1, HIGH);
  digitalWrite(MotorLpin2, LOW);
  delay(1000);
  digitalWrite(MotorLpin1, LOW);
  digitalWrite(MotorLpin2, LOW);
  delay(1000);
  digitalWrite(MotorLpin1, LOW);
  digitalWrite(MotorLpin2, HIGH);
  delay(1000);
  digitalWrite(MotorLpin1, LOW);
  digitalWrite(MotorLpin2, LOW);
  delay(1000);

}

