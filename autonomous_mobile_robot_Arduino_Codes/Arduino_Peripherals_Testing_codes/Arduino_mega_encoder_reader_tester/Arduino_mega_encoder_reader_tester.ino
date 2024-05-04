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
  // Attaching the ISR to encoder A
  attachInterrupt(digitalPinToInterrupt(EncRpinA), R_encoder_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EncLpinA), L_encoder_isr, CHANGE);
}
void loop() {
  Serial.println("Encoder value: " + String(L_Enc_val)+" "+ String(R_Enc_val));
  delay(10);
  
}