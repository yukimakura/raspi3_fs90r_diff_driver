#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import pigpio
import time
import math

gpio_pin0 = 19
gpio_pin1 = 18
rad = 0.035

pio = pigpio.pi()

def cmd_vel_CB(data):

    pio.set_mode(gpio_pin0, pigpio.OUTPUT)
    pio.set_mode(gpio_pin1, pigpio.OUTPUT)
    r_speed = data.linear.x + data.angular.z
    l_speed = data.linear.x - data.angular.z
    
    r_sec = 1.5 + 0.5*(r_speed/rad/(2*3.141592))
    l_sec = 1.5 - 0.5*(l_speed/rad/(2*3.141592))
    print("r_sec = "+str(r_sec))
    print("l_sec = "+str(l_sec))
    # GPIO18
    pio.set_servo_pulsewidth(gpio_pin0, int(r_sec*1000.0))
    # GPIO19
    pio.set_servo_pulsewidth(gpio_pin1, int(l_sec*1000.0))

def listener():

    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The 
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaenously.
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("cmd_vel", Twist, cmd_vel_CB)
    rospy.spin()
    # GPIO18
    pio.set_servo_pulsewidth(gpio_pin0, 1500)
    # GPIO19
    pio.set_servo_pulsewidth(gpio_pin1, 1500)
    pio.set_mode(gpio_pin0, pigpio.INPUT)
    pio.set_mode(gpio_pin1, pigpio.INPUT)
    pio.stop()
    print("ctr c")



        
if __name__ == '__main__':
    listener()

    