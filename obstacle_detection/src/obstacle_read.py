#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import numpy as np
import serial
import time


def click_callback(msg: LaserScan):
    
    try:
        grid1 = np.meshgrid(msg.ranges[:90],0.625,sparse=True) # 0.625 meters is object limit
        grid2 = np.meshgrid(msg.ranges[630:],0.625,sparse=True) # 0.625 meters is object limit
        
        if np.sum(grid1[0] < grid1[1]) > 1 or np.sum(grid2[0] < grid2[1]) > 1:
            rospy.loginfo("STOP")
            stop = '0'
            pub.publish(stop)

        else:
            rospy.loginfo("Go")
            #var = b'1'
            go = '1'
            pub.publish(go)

            
    except KeyboardInterrupt:
        ser.close() # when Ctrl-c happens close the serial port /dev/ttyAMA1
    

if __name__ == '__main__':
    rospy.init_node("obstacle_read") # node name

    sub = rospy.Subscriber("/scan", LaserScan, callback=click_callback)
    pub = rospy.Publisher('uart_tx_obstacle',String,queue_size=10)
    rospy.loginfo("Node has been started.")
    rospy.spin() # blocks until ROS node is shutdown
    
