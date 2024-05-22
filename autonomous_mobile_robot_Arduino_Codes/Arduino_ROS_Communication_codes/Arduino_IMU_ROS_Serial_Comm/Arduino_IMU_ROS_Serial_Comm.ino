#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_BNO080
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
//#define BNO080_DEFAULT_ADDRESS 0x4A

float quatI,quatJ,quatK,quatReal;
float gyroX,gyroY,gyroZ;
float accelX,accelY,accelZ;
 
// Define ROS node handle
ros::NodeHandle nh;

// Define the IMU message
std_msgs::Float32MultiArray imu_msg;
ros::Publisher imuPub("imu_raw", &imu_msg);

// Define BNO080 sensor
BNO080 myIMU;
void setup() {
  // Start serial communication
  Wire.begin();
  nh.initNode();
  //Serial.begin(9600);
  //Serial.println("BNO080 Read Example");   
  
  // Initialize the BNO080 sensor
  if (myIMU.begin() == false)
  {
    //Serial.println("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    nh.loginfo("IMU Initialization Failed....");
    while (1);
  }
  else
  {
    nh.loginfo("IMU Initialization Sucess....");    
  }

  Wire.setClock(400000); //Increase I2C data rate to 400kHz
  myIMU.enableRotationVector(50); //Send data update every 50ms
  myIMU.enableLinearAccelerometer(50);
  myIMU.enableGyro(50); 
  nh.advertise(imuPub);
  imu_msg.data_length = 10;// Initialize the data array
  imu_msg.data = (float*)malloc(sizeof(float) * imu_msg.data_length);
}

void loop() {

  // Check if new data is available
if (myIMU.dataAvailable() == true)
 {
    quatI = myIMU.getQuatI();
    quatJ = myIMU.getQuatJ();
    quatK = myIMU.getQuatK();
    quatReal = myIMU.getQuatReal();

    gyroX = myIMU.getGyroX();
    gyroY = myIMU.getGyroY();
    gyroZ = myIMU.getGyroZ();

    accelX = myIMU.getLinAccelX();
    accelY = myIMU.getLinAccelY();
    accelZ = myIMU.getLinAccelZ();
 }
 // Get the IMU data
  imu_msg.data[0] = quatI;
  imu_msg.data[1] = quatJ;
  imu_msg.data[2] = quatK;
  imu_msg.data[3] = quatReal;

  imu_msg.data[4] = gyroX;
  imu_msg.data[5] = gyroY;
  imu_msg.data[6] = gyroZ;

  imu_msg.data[7] = accelX;
  imu_msg.data[8] = accelY;
  imu_msg.data[9] = accelZ;

  imuPub.publish(&imu_msg);
  // Handle ROS communication
  nh.spinOnce();
  delay(10);
}