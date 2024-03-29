#include <ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>

const uint8_t R_PWM_Pin =  7;
const uint8_t R_FORW_Pin = 50;
const uint8_t R_BACK_Pin = 48;

const uint8_t L_PWM_Pin =  6;
const uint8_t L_FORW_Pin = 46;
const uint8_t L_BACK_Pin = 44;

float left_wheel;
float right_wheel; 

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

void pin_def()
{
  pinMode(L_PWM_Pin,  OUTPUT);
  pinMode(L_FORW_Pin, OUTPUT);
  pinMode(L_BACK_Pin, OUTPUT);
  pinMode(R_PWM_Pin,  OUTPUT);
  pinMode(R_FORW_Pin, OUTPUT);
  pinMode(R_BACK_Pin, OUTPUT);
 
  // Set pin states of the encoder
  pinMode(ENC_IN_LEFT_A , INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B , INPUT);
  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B , INPUT);
 
  // Every time the pin goes high, this is a tick
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_tick, RISING);
 }
 
void drive()
{   
  if(left_wheel==right_wheel)
  {
   analogWrite(R_PWM_Pin, 180);  
   analogWrite(L_PWM_Pin, 180);
  }
  else   
  {
   analogWrite(R_PWM_Pin, 150);  
   analogWrite(L_PWM_Pin, 150);    
  }   
  
  digitalWrite(L_FORW_Pin,left_wheel > 0);
  digitalWrite(L_BACK_Pin,left_wheel < 0);
  digitalWrite(R_FORW_Pin,right_wheel > 0);
  digitalWrite(R_BACK_Pin,right_wheel < 0);
  
}

void stop()
{   digitalWrite(L_FORW_Pin, 0);
    digitalWrite(L_BACK_Pin, 0);
    digitalWrite(R_FORW_Pin, 0);
    digitalWrite(R_BACK_Pin, 0);
    digitalWrite(R_PWM_Pin, 0);  
    digitalWrite(L_PWM_Pin, 0);    
}

void cmdVel_cb( const geometry_msgs::Twist& velocity_msg){

    left_wheel = (velocity_msg.linear.x - velocity_msg.angular.z ) / 2 ;
    right_wheel = (velocity_msg.linear.x + velocity_msg.angular.z ) /2 ;
    drive();
    if ( velocity_msg.linear.x ==0.0 & velocity_msg.angular.z ==0.0)
    {
        stop();
    }
}

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

// Handles startup and shutdown of ROS
ros::NodeHandle nh;

// Keep track of the number of wheel ticks
std_msgs::Int16 right_wheel_tick_count;
ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);
 
std_msgs::Int16 left_wheel_tick_count;
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &cmdVel_cb );
 
void setup() {

  pin_def();
  stop();
  // ROS Setup
  nh.initNode();
  nh.advertise(rightPub);
  nh.advertise(leftPub);
  nh.subscribe(sub);
  delay(2000);

}
 
void loop() {
    
    right_wheel_tick_count.data = R_Enc_val ;    
    left_wheel_tick_count.data = L_Enc_val ;
    rightPub.publish( &right_wheel_tick_count );
    leftPub.publish( &left_wheel_tick_count );
    nh.spinOnce();
    delay(10);
}