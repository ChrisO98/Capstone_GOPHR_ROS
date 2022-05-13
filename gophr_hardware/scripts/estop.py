#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Input is pin 13 (GPIO 27), Output is pin 11 (GPIO 17)
out_estop = 17
in_estop = 27

GPIO.setup(out_estop, GPIO.OUT)
GPIO.setup(in_estop, GPIO.IN, pull_up_down=GPIO.PUD_UP)

rospy.init_node("estop_node") # node name
rospy.loginfo("Node has been started.")
pub_estop = rospy.Publisher('uart_tx_obstacle', String, queue_size=10) # may publish but not

while not rospy.is_shutdown():
    # if estop is pressed send a 1 or 0 to hero board depends on what side of estop is connected
    GPIO.output(out_estop, 1) # set GPIO 27 to 1/GPIO.HIGH/True
    if GPIO.input(in_estop)==0:
        print("Estop Pressed, pin 13 (GPIO 27)")
        msg = String()
        msg.data = '2'
        pub_estop.publish(msg)


