const int EnRpin=7;
const int MotorRpin1 = 50;
const int MotorRpin2 = 48;

const int EnLpin=6;
const int MotorLpin1 = 46;
const int MotorLpin2 = 44;
 
const int EncRpinA=2;
const int EncRpinB=4;
const int EncLpinA=3;
const int EncLpinB=5;

volatile int L_Enc_val = 0; // Global variable for storing the encoder position
volatile int R_Enc_val = 0; // Global variable for storing the encoder position

void L_encoder_isr() {
  // Reading the current state of encoder A and B
  int L_EncA = digitalRead(EncLpinA);
  int L_EncB = digitalRead(EncLpinB);
  // If the state of A changed, it means the encoder has been rotated
  if ((L_EncA == HIGH) != (L_EncB == LOW)) {
    L_Enc_val--;
  } else {
    L_Enc_val++;
  }
}
void R_encoder_isr() {
  // Reading the current state of encoder A and B
  int R_EncA = digitalRead(EncRpinA);
  int R_EncB = digitalRead(EncRpinB);
  // If the state of A changed, it means the encoder has been rotated
  if ((R_EncA == HIGH) != (R_EncB == LOW)) {
    R_Enc_val--;
  } else {
    R_Enc_val++;
  }
}
void setup() {
  Serial.begin(115200); // Initialize serial communication
  pinMode(EncRpinA, INPUT);
  pinMode(EncRpinB, INPUT);
  pinMode(EncLpinA, INPUT);
  pinMode(EncLpinB, INPUT);
  pinMode(EnRpin, OUTPUT);
  pinMode(EnLpin, OUTPUT);
  pinMode(MotorRpin1, OUTPUT);
  pinMode(MotorRpin2, OUTPUT);
  pinMode(MotorLpin1, OUTPUT);
  pinMode(MotorLpin2, OUTPUT);
  analogWrite(EnRpin, 220);
  analogWrite(EnLpin, 220);
  // Attaching the ISR to encoder A
  attachInterrupt(digitalPinToInterrupt(2), R_encoder_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(3), L_encoder_isr, RISING);
}
void loop() {
  Serial.println("Encoder value: " + String(L_Enc_val)+" "+ String(R_Enc_val));
  delay(10);
  digitalWrite(MotorRpin1, HIGH);
  digitalWrite(MotorRpin2, LOW);
  digitalWrite(MotorLpin1, HIGH);
  digitalWrite(MotorLpin2, LOW);
  delay(1000);
  
  digitalWrite(MotorRpin1, LOW);
  digitalWrite(MotorRpin2, LOW);
  digitalWrite(MotorLpin1, LOW);
  digitalWrite(MotorLpin2, LOW);
  delay(1000);
  
  Serial.println("Encoder value: " + String(L_Enc_val)+" "+ String(R_Enc_val));
  delay(10);
  digitalWrite(MotorRpin1, LOW);
  digitalWrite(MotorRpin2, HIGH);
  digitalWrite(MotorLpin1, LOW);
  digitalWrite(MotorLpin2, HIGH);
  delay(1000);
  digitalWrite(MotorRpin1, LOW);
  digitalWrite(MotorRpin2, LOW);
  digitalWrite(MotorLpin1, LOW);
  digitalWrite(MotorLpin2, LOW);
  delay(1000);
  
}