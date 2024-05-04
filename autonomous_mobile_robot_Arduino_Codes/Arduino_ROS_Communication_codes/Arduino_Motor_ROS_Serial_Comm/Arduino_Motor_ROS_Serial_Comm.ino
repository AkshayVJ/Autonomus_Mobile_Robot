#include <ros.h>
#include <geometry_msgs/Twist.h>

const uint8_t R_PWM_Pin =  7;
const uint8_t R_FORW_Pin = 50;
const uint8_t R_BACK_Pin = 48;

const uint8_t L_PWM_Pin =  6;
const uint8_t L_FORW_Pin = 46;
const uint8_t L_BACK_Pin = 44;

float left_wheel;
float right_wheel;

// 100ms interval for measurements
const int interval = 100;
long previousMillis = 0;
long currentMillis = 0;

void pin_def()
{
  pinMode(L_PWM_Pin,  OUTPUT);
  pinMode(L_FORW_Pin, OUTPUT);
  pinMode(L_BACK_Pin, OUTPUT);
  pinMode(R_PWM_Pin,  OUTPUT);
  pinMode(R_FORW_Pin, OUTPUT);
  pinMode(R_BACK_Pin, OUTPUT);
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
    Serial.print(left_wheel);Serial.print(" / ");Serial.println(right_wheel);
}

ros::NodeHandle  nh;
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &cmdVel_cb );

void setup()
{ 
    Serial.begin(115200);
    nh.initNode();
    nh.subscribe(sub);
    pin_def();
    stop();
    Serial.println("Get Ready");
    delay(2000);
}

void loop()
{
// Record the time
  currentMillis = millis();
 
  // If 100ms have passed, print the number of ticks
  if (currentMillis - previousMillis > interval) 
  {  
  previousMillis = currentMillis;
  nh.spinOnce();
  }  
  
}

