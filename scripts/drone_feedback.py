#!/usr/bin/env python

## 
# @file drone_feedback.py 
# @author Davide Piccinini <dav.piccinini\@gmail.com>
# @date 9 February 2022
# @brief This script shows on the terminal the feedback about the drone
#        state: the displayed information comprises the drone's linear and
#        angular velocity and pose with respect to the world frame, and the
#        distance and the qualitative position of the closest obstacle.

import rospy
import math #
import os #
import threading
import csv #
import tf_conversions
from nav_msgs.msg import Odometry
from interface_code.msg import ClosestObstacleInfo

## Ratio to convert rad to deg
radToDegRatio = 180 / math.pi

## Threading event
odometryReceived = threading.Event()
## Threading event
infoReceived = threading.Event()

## Odometry variable
dronePosition = None
## Odometry variable
droneAttitudeRad = None
## Odometry variable
droneAttitudeRad = None
## Odometry variable
dronePosition = None
## Odometry variable
dronePosition = None


##
# @brief The callback function for the @c '/ground_truth/state' topic.
# 
# The function 
#
# @param scan The Odometry message
def odometry_callback(odometryMsg):

    # Process the message only when the previous one has been logged
    if not odometryReceived.is_set():
        dronePosition = odometryMsg.pose.pose.position
        droneAttitudeRad = tf_conversions.transformations.euler_from_quaternion([odometryMsg.pose.pose.orientation.x, odometryMsg.pose.pose.orientation.y, odometryMsg.pose.pose.orientation.z, odometryMsg.pose.pose.orientation.w])
        droneAttitudeDeg = [value * radToDegRatio for value in droneAttitudeRad]
        droneLinVel = odometryMsg.twist.twist.linear
        droneAccVel = odometryMsg.twist.twist.angular
        
        # Set the flag to true
        odometryReceived.set()


##
# @brief The callback function for the @c '/closest_obstacle_info' topic.
# 
# The function 
#
# @param scan The ClosestObstacleInfo message
def obstacle_info_callback(infoMsg):

    # Process the message only when the previous one has been logged
    if not infoReceived.is_set():
        
        
        # Set the flag to true
        infoReceived.set()
    

##
# Main function
if __name__ == '__main__':

	try:
		# Initialize the node
		rospy.init_node('drone_feedback')
    	
    	# Subscribe to the "/ground_truth/state" topic
		rospy.Subscriber('/ground_truth/state', Odometry, odometry_callback)
		
    	# Subscribe to the "/closest_obstacle_info" topic
		rospy.Subscriber('/closest_obstacle_info', ClosestObstacleInfo, obstacle_info_callback)
    
		while not rospy.is_shutdown():
            # Keep from exiting until the node is stopped
			rospy.spin()
        
	except rospy.ROSInterruptException:
		pass
