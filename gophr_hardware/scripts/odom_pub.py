#!/usr/bin/env python3

# This file is not in use but could be implemented and 
#  replace laser scan matcher with data from wheel encoders

from math import sin, cos, pi

import rospy
import tf, tf.transformations
from nav_msgs.msg import Odometry
#from std_msgs.msg import Float64
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import serial
import time

# testing receiving odometry from hero board
#port = "/dev/ttyAMA1"
#ser = serial.Serial(port, baudrate = 115200,timeout = 0) # opens port /dev/ttyAMA1

#data = ser.read(24) # works, get a lot of data from hero, may be tricky
#data_left = ser.inWaiting() #check for remaining byte
#data += ser.read(data_left) # add remaining byte
#read_data = data.decode("utf-8") # change data from binary to string

#set_vx_first = float(read_data[0]) # first num is int, vx
#set_vx_last = set_x_first + float(read_data[1:4])/1000 # vx decimals
#set_vy_first = float(read_data[4]) # first num is int, vy
#set_vy_last = set_x_first + float(read_data[5:8])/1000 # vy decimals
#set_vth_first = float(read_data[8]) # first num is int, vth
#set_vth_last = set_x_first + float(read_data[9:12])/1000 # vth decimals

rospy.init_node('odometry_publisher')

odom_pub = rospy.Publisher("odom", Odometry, queue_size=100) #queue_size=50
#msg: Vector3)
#def odom_call():
odom_broadcaster = tf.TransformBroadcaster()

x = 0.0
y = 0.0
th = 0.0

    #vx = set_vx_last
    #vy = set_vy_last
    #vth = set_vth_last
    
vx = 0.0
vy = 0.0
vth = 0.0

    #vx = msg.x
    #vy = msg.y
    #vth = msg.z

current_time = rospy.Time.now()
last_time = rospy.Time.now()

r = rospy.Rate(10.0)
while not rospy.is_shutdown():
    current_time = rospy.Time.now()

    # compute odometry in a typical way given the velocities of the robot
    dt = (current_time - last_time).to_sec()
    delta_x = (vx * cos(th) - vy * sin(th)) * dt
    delta_y = (vx * sin(th) + vy * cos(th)) * dt
    delta_th = vth * dt

    x += delta_x
    y += delta_y
    th += delta_th

    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

    # first, we'll publish the transform over tf
    #odom_broadcaster.sendTransform(
    #    (x, y, 0.0),
    #    odom_quat,
    #    current_time,
    #    "base_footprint",
    #    "odom"
    #) # robot_pose_ekf will handle the odom transform frame, it needs data from odom topic to work

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = "base_footprint"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))
    
    covariance_x = 0.000100
    covariance_y = 0.000100
    covariance_yaw = 0.010000

    # set covariance
    odom.pose.covariance[0] = covariance_x
    odom.pose.covariance[7] = covariance_y
    odom.pose.covariance[14] = 1000000000000.0
    odom.pose.covariance[21] = 1000000000000.0
    odom.pose.covariance[28] = 1000000000000.0
    odom.pose.covariance[35] = covariance_yaw

    odom.twist.covariance[0] = covariance_x
    odom.twist.covariance[7] = covariance_y
    odom.twist.covariance[14] = 1000000000000.0
    odom.twist.covariance[21] = 1000000000000.0
    odom.twist.covariance[28] = 1000000000000.0
    odom.twist.covariance[35] = covariance_yaw

    # publish the message
    odom_pub.publish(odom)

    last_time = current_time
    r.sleep()

#if __name__ == "__main__":
    #rospy.init_node('odometry_publisher')

    #odom_pub = rospy.Publisher("odom", Odometry, queue_size=100) #queue_size=50
    #sub_odom = rospy.Subscriber('uart_odom_pub', Vector3, callback=odom_call)
#    odom_call()
#    rospy.loginfo("Node has been started.")
#    rospy.spin() # blocks until ROS node is shutdown
