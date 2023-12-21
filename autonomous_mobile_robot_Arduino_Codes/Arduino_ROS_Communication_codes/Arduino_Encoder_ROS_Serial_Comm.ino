#include <ros.h>
#include <std_msgs/Int16.h>
 
// Encoder output to Arduino Interrupt pin. Tracks the tick count.
const int ENC_IN_RIGHT_A = 2;
const int ENC_IN_LEFT_A = 3;
const int ENC_IN_RIGHT_B = 4;
const int ENC_IN_LEFT_B = 5;

 
// True = Forward; False = Reverse
boolean Direction_left = true;
boolean Direction_right = true;
 
// Minumum and maximum values for 16-bit integers
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;

volatile int L_Enc_val = 0; // Global variable for storing the encoder position
volatile int R_Enc_val = 0; // Global variable for storing the encoder position

// Handles startup and shutdown of ROS
ros::NodeHandle nh;

// Keep track of the number of wheel ticks
std_msgs::Int16 right_wheel_tick_count;
ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);
 
std_msgs::Int16 left_wheel_tick_count;
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);
 
// Increment the number of ticks
void right_wheel_tick() {
   
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_RIGHT_B);
 
  if(val == LOW) {
    Direction_right = false; // Reverse
  }
  else {
    Direction_right = true; // Forward
  }
   
  if (Direction_right) {
     
    if (R_Enc_val == encoder_maximum) {
      R_Enc_val = encoder_minimum;
    }
    else {
      R_Enc_val++;  
    }    
  }
  else {
    if (R_Enc_val == encoder_minimum) {
      R_Enc_val = encoder_maximum;
    }
    else {
      R_Enc_val--;  
    }   
  }
}
 
// Increment the number of ticks
void left_wheel_tick() {
   
  // Read the value for the encoder for the left wheel
  int val = digitalRead(ENC_IN_LEFT_B);
 
  if(val == LOW) {
    Direction_left = true; // Reverse
  }
  else {
    Direction_left = false; // Forward
  }
   
  if (Direction_left) {
    if (L_Enc_val == encoder_maximum) {
      L_Enc_val = encoder_minimum;
    }
    else {
      L_Enc_val++;  
    }  
  }
  else {
    if (L_Enc_val == encoder_minimum) {
      L_Enc_val = encoder_maximum;
    }
    else {
      L_Enc_val--;
        
    }   
  }
  
}
 
void setup() {
 
  // Set pin states of the encoder
  pinMode(ENC_IN_LEFT_A , INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B , INPUT);
  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B , INPUT);
 
  // Every time the pin goes high, this is a tick
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_tick, RISING);
 
  // ROS Setup
  nh.initNode();
  nh.advertise(rightPub);
  nh.advertise(leftPub);
}
 
void loop() {
    
    right_wheel_tick_count.data = R_Enc_val ;    
    left_wheel_tick_count.data = L_Enc_val ;
    rightPub.publish( &right_wheel_tick_count );
    leftPub.publish( &left_wheel_tick_count );
    nh.spinOnce();
    delay(1000);
}