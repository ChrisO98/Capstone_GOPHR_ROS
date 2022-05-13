#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from gpiozero import AngularServo
import time


def servo_lock(msg: String):
    # lock or unlock
    print(msg.data)
    global servo
    if msg.data == 'Unlock':
        servo.angle = -20
        time.sleep(3)
        servo.angle = -54



if __name__ == "__main__":
    rospy.init_node("servo_lock_node") # node name
    servo = AngularServo(18, min_pulse_width=0.0005, max_pulse_width=0.003)
    servo.angle = -54
    
    # set subscriber
    sub_servo = rospy.Subscriber('servo_lock', String, callback=servo_lock)
    
    rospy.loginfo("Node has been started.")
    rospy.spin() # blocks until ROS node is shutdown

