#!/usr/bin/env python3

import time
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray


def imuDataCallback(data):

    global quat_x
    global quat_y
    global quat_z
    global quat_w
    global gyro_x
    global gyro_y 
    global gyro_z 
    global accel_x 
    global accel_y
    global accel_z 

    quat_x = data.data[0]
    quat_y = data.data[1]
    quat_z = data.data[2]
    quat_w = data.data[3]
    gyro_x = data.data[4]
    gyro_y = data.data[5]
    gyro_z = data.data[6]
    accel_x = data.data[7]
    accel_y = data.data[8]
    accel_z = data.data[9]


def imu_publisher():

    global quat_x
    global quat_y
    global quat_z
    global quat_w
    global gyro_x
    global gyro_y 
    global gyro_z 
    global accel_x 
    global accel_y
    global accel_z 

    quat_x=0
    quat_y=0
    quat_z=0
    quat_w=0
    gyro_x=0
    gyro_y=0
    gyro_z=0 
    accel_x=0 
    accel_y=0
    accel_z=0 

# Initialize ROS node
    rospy.init_node('Imu_publisher', anonymous=True)
    imu_pub = rospy.Publisher('/imu', Imu, queue_size=10)
    imu_sub = rospy.Subscriber('/imu_raw', Float32MultiArray, imuDataCallback)

# Define an Imu message
    imu_msg = Imu()
    imu_msg.header.frame_id = "imu"

    while not rospy.is_shutdown():


        # Update the Imu message
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.orientation.x =quat_x
        imu_msg.orientation.y = quat_y
        imu_msg.orientation.z = quat_z
        imu_msg.orientation.w = quat_w

        imu_msg.angular_velocity.x = gyro_x
        imu_msg.angular_velocity.y = gyro_y 
        imu_msg.angular_velocity.z = gyro_z
        
        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = accel_y
        imu_msg.linear_acceleration.z = accel_z
        
        # Publish the message
        imu_pub.publish(imu_msg)

        # Sleep to maintain the loop rate
        time.sleep(0.1)

if __name__ == '__main__':       
    try:
        imu_publisher()
    except rospy.ROSInterruptException:
        pass
    
