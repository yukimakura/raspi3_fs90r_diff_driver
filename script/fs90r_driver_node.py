#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import pigpio
import time
import math

gpio_pin0 = 19
gpio_pin1 = 18
rad = 0.035
old_x = None
old_yaw = None

pio = pigpio.pi()

def cmd_vel_CB(data):
    global old_x
    global old_yaw
    pio.set_mode(gpio_pin0, pigpio.OUTPUT)
    pio.set_mode(gpio_pin1, pigpio.OUTPUT)
    r_speed = data.linear.x + data.angular.z
    l_speed = data.linear.x - data.angular.z
    
    # print("old_x = "+str(old_x))
    # if old_x != data.linear.x or old_yaw != data.angular.z :
    r_sec = 1.5 + 0.5*(r_speed/rad/(2*3.141592))
    l_sec = 1.5 - 0.5*(l_speed/rad/(2*3.141592))
    print("r_sec = "+str(r_sec))
    print("l_sec = "+str(l_sec))
    # GPIO18
    pio.hardware_PWM(gpio_pin0, 50, int(1000000*r_sec/20))
    # GPIO19
    pio.hardware_PWM(gpio_pin1, 50, int(1000000*l_sec/20))
    old_x = data.linear.x
    old_yaw = data.angular.y

def listener():

    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The 
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaenously.
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("cmd_vel", Twist, cmd_vel_CB)
    global old_x
    global old_yaw
    old_x = 0.0
    old_yaw = 0.0
    rospy.spin()
    # GPIO18
    pio.hardware_PWM(gpio_pin0, 50, int(1000000*1.5/20))
    # GPIO19
    pio.hardware_PWM(gpio_pin1, 50, int(1000000*1.25/20))
    pio.set_mode(gpio_pin0, pigpio.INPUT)
    pio.set_mode(gpio_pin1, pigpio.INPUT)
    pio.stop()
    print("ctr c")



        
if __name__ == '__main__':
    listener()

    