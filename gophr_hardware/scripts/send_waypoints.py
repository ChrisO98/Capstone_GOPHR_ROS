#!/usr/bin/env python3

import rospy
import math

import actionlib
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Int16, Float32, Float32MultiArray
from gophr_hardware.msg import Float32Array


class waypoint():

    def __init__(self):

        rospy.init_node('move_base_waypoints')

        # only subscribe for an array of data from main.py ui then do all of the listing
        # Send a 1 for 'room one' and do an integer string concatnate to use waypoint1.csv
        #  example: ("...\waypointNodes%d", x=1)

        rospy.Subscriber("/room_number", Int16, callback=self.click_callback) # recieve from user_interface main.py
        
        

    
    def click_callback(self, msg: Int16): # change pose to just an integer
        #points_seq = [message.position.x, message.position.y, message.position.z] # recieve the coordinates from coord in a message of type Pose
        wayponint = "/home/tjcc/catkin_ws/src/user_interface/saved_waypoints/waypoints%d.csv" % msg.data # number concatnate
        #f = open("/home/tjcc/catkin_ws/src/gophr/nav_scripts/waypointNodes.csv","r")
        f = open(wayponint,"r")
        rawNodes = f.readlines()
        f.close()
        nodes = []
        for i in rawNodes:
            #nodes.append([float(i.split(",")[0]), float(i.split(",")[1])])
            nodes.append(float(i.split(",")[0])) # x
            nodes.append(float(i.split(",")[1])) # y
            #nodes.append(0.0) # z
            #rospy.loginfo(nodes)
        
        if len(nodes) < 2:
            print("Not enough waypoinys")
            exit()

        points_seq = nodes

        # Returns a list of lists [[point1], [point2],...[pointn]]
        points = [points_seq[i:i+2] for i in range(0, len(points_seq), 2)]

        yawrad_seq = []
        for i in rawNodes:
            yawrad_seq.append([0.0,0.0,float(i.split(",")[2]), float(i.split(",")[3])]) # angle z,w
            
        # convert quaternion (z,w) to Euler
        angles = []
        for i,val in enumerate(yawrad_seq):
            (roll, pitch, yaw) = euler_from_quaternion(yawrad_seq[i])
            angles.append(yaw*180/math.pi)
        
        angle_size = len(angles)
        waypoint_data = []
        angle_index = 0
        for i in points:
            for value in i:
                waypoint_data.append(value)
            if angle_index < len(angles):
                waypoint_data.append(angles[angle_index])
                angle_index += 1

        #print(waypoint_data)

        # UART PUBLISH
        pub.publish(waypoint_data)
    
    
    

if __name__ == '__main__':
    try:
        pub = rospy.Publisher('uart_tx_waypoint', Float32Array, queue_size=10)
        waypoint()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")

