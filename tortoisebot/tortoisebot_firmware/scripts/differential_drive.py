#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Bool
import time
from math import pi


motor_rpm = 185  #   max rpm of motor on full voltage
wheel_diameter = 0.06  #   in meters
wheel_separation = 0.2 #   in meters
max_pwm_val = 250  #   Maximum PWM value 255 for Arduino
min_pwm_val = 150  #   Minimum PWM value that is needed for the robot to move

wheel_radius = wheel_diameter / 2
circumference_of_wheel = 2 * pi * wheel_radius
max_speed = (circumference_of_wheel * motor_rpm) / 60  #   m/sec
max_ang = (max_speed * 2) / wheel_separation  #   rad/sec


def wheel_vel_executer(left_speed, right_speed):

    global max_pwm_val
    global min_pwm_val

    global lpwm_pub
    global rpwm_pub
    global ldir_pub
    global rdir_pub

    if(left_speed==0.0 and right_speed ==0.0):
    
        lspeedPWM=0
        rspeedPWM=0 
    else:

        lspeedPWM = max(min(((abs(left_speed) / max_speed) * max_pwm_val), max_pwm_val), min_pwm_val)
        rspeedPWM = max(min(((abs(right_speed) / max_speed) * max_pwm_val), max_pwm_val), min_pwm_val)

    lpwm=int(lspeedPWM)
    rpwm=int(rspeedPWM)

    lpwm_pub.publish(lpwm)    
    rpwm_pub.publish(rpwm)

    if left_speed > 0 and right_speed > 0:
        ldir_pub.publish(1)
        rdir_pub.publish(1)
    elif left_speed < 0 and right_speed < 0:
        ldir_pub.publish(-1)
        rdir_pub.publish(-1)
    elif left_speed < 0 and right_speed > 0:
        ldir_pub.publish(0)
        rdir_pub.publish(1)
    elif left_speed > 0 and right_speed < 0:
        ldir_pub.publish(1)
        rdir_pub.publish(0)
    else:
        ldir_pub.publish(0)
        rdir_pub.publish(0)

    
def callback(data):

    global wheel_radius
    global wheel_separation

    linear_vel = data.linear.x  # Linear Velocity of Robot
    angular_vel = data.angular.z  # Angular Velocity of Robot


    VrplusVl = 2 * linear_vel
    VrminusVl = angular_vel * wheel_separation

    right_vel = (VrplusVl + VrminusVl) / 2  # right wheel velocity along the ground
    left_vel = VrplusVl - right_vel  # left wheel velocity along the ground

    wheel_vel_executer(left_vel, right_vel)


def differential_drive():
    global lpwm_pub
    global rpwm_pub
    global ldir_pub
    global rdir_pub

    rospy.init_node("cmdvel_subscriber", anonymous=False)
    rospy.Subscriber("/cmd_vel", Twist, callback)
    lpwm_pub = rospy.Publisher("left_vel", Int32, queue_size=10)
    rpwm_pub = rospy.Publisher("right_vel", Int32, queue_size=10)
    ldir_pub = rospy.Publisher("left_dir", Int32, queue_size=10)
    rdir_pub = rospy.Publisher("right_dir", Int32, queue_size=10)
    rospy.spin()


if __name__ == "__main__":
    try:
        differential_drive()
    except rospy.ROSInterruptException:
        pass
