#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PointStamped, Pose, PoseStamped

# Save more than two waypoints when using rviz and the tool 2D Nav Goal

def click_callback(msg: PoseStamped): ## message is a point used to be PointStamped
    # Creates a .csv file and saves waypoint data into the .csv file
    f = open("/home/tjcc/catkin_ws/src/user_interface/saved_waypoints/waypoints.csv","a+")
    #f.write(str(msg.point.x)+","+str(msg.point.y)+"\n") # may need to add yaw
    f.write(str(msg.pose.position.x)+","+str(msg.pose.position.y)+","+str(msg.pose.orientation.z)+","+str(msg.pose.orientation.w)+"\n")
    f.close()
    
    print("Data:")
    print("X: " + str(msg.pose.position.x))
    print("Y: " + str(msg.pose.position.y))
    print("Z: " + str(msg.pose.orientation.z))
    print("W: " + str(msg.pose.orientation.w))



if __name__ == '__main__':
    rospy.init_node("coordinates_grab") # node name
    # initialize .csv file
    f = open("/home/tjcc/catkin_ws/src/user_interface/saved_waypoints/waypoints.csv","w+")
    f.close()
    sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, callback=click_callback)

    rospy.loginfo("Node has been started.")

    rospy.spin() # blocks until ROS node is shutdown
    
