#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, UInt16, Float32MultiArray
from geometry_msgs.msg import Vector3
from gophr_hardware.msg import Float32Array
import serial
import time
import struct
import array as arr


# UART is used to send a single byte of data between the HERO and Pi


# create a class and create varialbes for the port and serial open
# this would result in a static variable and functions in the class can use it
# this would remove the global type

def uart_transmit_waypoint(msg: String):
    # send waypoints
    global port, ser
    
    # convert from float to int to byte
    temp_value = []
    byte_array = []
            
    # convert from float to str to byte
    for i in msg.data:
        str_val = str(int(i*1000000))
        byte_array.append(str.encode(str_val))
    
    print(byte_array)
    
    # Read from hero
    start = 0
    end = 3
    
    len_waypoint = str(int(len(byte_array)/3))
    print(len_waypoint)
    ser.write(str.encode(len_waypoint))
    i = 0
    while i < len(byte_array):
        x = ser.read(1)
        print(x)
        #time.sleep(1)
        if x == b'\x01':
            send_data = byte_array[i]
            print(send_data)
            ser.write(send_data)
            i = i + 1

def uart_transmit_stop(msg: String):
    # send stop
    global port, ser, home
    
    if msg.data == '2':
        estop = b'0'
        ser.reset_output_buffer()
        ser.write(estop)
        ser.reset_input_buffer()
        print("OH NO ESTOP!")
        
    
    elif msg.data == '0':
        #stop = b'0'
        x = ser.read(1)
        
        if x == b'\x04':
            # set global variavle to allow home return when nurse tell robot to go home
            home = 0
            home_done = String()
            home_done.data = 'base'
            pub_home_done.publish(home_done)
            print("We are home.")
            ser.reset_output_buffer()
            stop = b'0'
            ser.reset_input_buffer()
        
        if x == b'\x03':
            # set global variavle to allow home return when nurse tell robot to go home
            home = 1
            path_done = String()
            path_done.data = 'done'
            pub_path_complete.publish(path_done)
            print("We go home?")
            ser.reset_output_buffer()
            stop = b'0'
            ser.reset_input_buffer()
            
        if x == b'\x02': # hero board sends this coninuously
            stop = b'0'
            ser.reset_output_buffer()
            ser.write(stop)
            ser.reset_input_buffer()
            
        ser.reset_input_buffer()

        
    else:
        x = ser.read(1)
        
        if x == b'\x04':
            # set global variavle to allow home return when nurse tell robot to go home
            home = 1
            home_done = String()
            home_done.data = 'base'
            pub_home_done.publish(home_done)
            print("We are home.")
            ser.reset_output_buffer()
            go = b'0'
            ser.reset_input_buffer()
        
        if x == b'\x03':
            # set global variavle to allow home return when nurse tell robot to go home
            home = 1
            path_done = String()
            path_done.data = 'done'
            pub_path_complete.publish(path_done)
            print("We go home?")
            ser.reset_output_buffer()
            go = b'0'
            ser.reset_input_buffer()
        
        if x == b'\x02': # hero board sends this coninuously
            go = b'1'
            ser.reset_output_buffer()
            ser.write(go)
            ser.reset_input_buffer()
            

        print(x)
        ser.reset_input_buffer()
        

def go_home(msg: String):
    global port, ser, home
    if msg.data == 'home' and home == 1:
        ser.write(b'1')
        print("Going to home base")
        home = 0



if __name__ == "__main__":
    rospy.init_node("uart_node") # node name
    
    # Initialize UART
    port = "/dev/ttyAMA1"
    ser = serial.Serial(port, baudrate = 9600,timeout = 0) # opens port /dev/ttyAMA1
    
    home = 0
    
    # set subscribers and publisher
    sub_tx_waypoint = rospy.Subscriber('uart_tx_waypoint', Float32Array, callback=uart_transmit_waypoint)
    
    sub_tx_stop = rospy.Subscriber('uart_tx_obstacle', String, callback=uart_transmit_stop)
    
    rospy.loginfo("Node has been started.")
    
    sub_home = rospy.Subscriber('uart_home', String, callback=go_home)
    pub_path_complete = rospy.Publisher('path_complete', String, queue_size=10)
    pub_home_done = rospy.Publisher('home_base', String, queue_size=10)
    
    rospy.spin() # blocks until ROS node is shutdown

