#!/usr/bin/env python

## 
# @file drone_feedback.py 
# @author Davide Piccinini <dav.piccinini\@gmail.com>
# @date 9 February 2022
# @brief This script shows on the terminal the feedback about the drone
#        state: the displayed information comprises the drone's linear and
#        angular velocities, pose with respect to the world frame, and the
#        distance and the qualitative position of the closest obstacle. The
#        data is also logged in a csv file.

import rospy
import math
import os
import threading
import csv
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
droneAttitudeDeg = None
## Odometry variable
droneLinVel = None
## Odometry variable
droneAngVelDeg = None

## Obstacle info variable
obstacleDistance = None
## Obstacle info variable
obstaclePosition = None


##
# @brief The callback function for the @c '/ground_truth/state' topic.
#
# @param odometryMsg The Odometry message.
def odometry_callback(odometryMsg):
    global dronePosition, droneAttitudeDeg, droneLinVel, droneAngVelDeg
    
    # Process the message only when the previous one has been logged
    if not odometryReceived.is_set():
        dronePosition = odometryMsg.pose.pose.position
        
        # Convert the attitude to degrees
        droneAttitudeRad = tf_conversions.transformations.euler_from_quaternion([odometryMsg.pose.pose.orientation.x, odometryMsg.pose.pose.orientation.y, odometryMsg.pose.pose.orientation.z, odometryMsg.pose.pose.orientation.w])
        droneAttitudeDeg = [value * radToDegRatio for value in droneAttitudeRad]
        
        droneLinVel = odometryMsg.twist.twist.linear
        
        # Convert the angular velocities to degrees per second
        droneAngVelDeg = odometryMsg.twist.twist.angular
        droneAngVelDeg.x *= radToDegRatio
        droneAngVelDeg.y *= radToDegRatio
        droneAngVelDeg.z *= radToDegRatio
        
        # Set the flag to true
        odometryReceived.set()


##
# @brief The callback function for the @c '/closest_obstacle_info' topic.
# 
# @param infoMsg The ClosestObstacleInfo message.
def obstacle_info_callback(infoMsg):
    global obstacleDistance, obstaclePosition
    
    # Process the message only when the previous one has been logged
    if not infoReceived.is_set():
        obstacleDistance = infoMsg.distance
        obstaclePosition = infoMsg.position
        
        # Set the flag to true
        infoReceived.set()


##
# @brief This function simply prints the important information about the
#        drone state in an user-friendly way.
def print_feedback():
    global dronePosition, droneAttitudeDeg, droneLinVel, droneAngVelDeg
    global obstacleDistance, obstaclePosition
    
    # Print on the terminal the information that is useful to the pilot
    print("The current position (in meters) of the drone is:")
    print("* x: " + str(round(dronePosition.x, 2)) + "\n* y: " + str(round(dronePosition.y, 2)) + "\n* z: " + str(round(dronePosition.z, 2)))
    print
    print("The current yaw attitude (in degrees) of the drone is:")
    print("* Yaw: " + str(round(droneAttitudeDeg[2], 2)))
    print
    print("The current linear velocity (in m/s) of the drone is:")
    print("* Velocity in x: " + str(round(droneLinVel.x, 2)) + "\n* Velocity in y: " + str(round(droneLinVel.y, 2)) + "\n* Velocity in z: " + str(round(droneLinVel.z, 2)))
    print
    print("The current yaw angular velocity (in deg/s) of the drone is:")
    print("* Rotation about the Z axis: " + str(round(droneAngVelDeg.z, 2)))
    print
    print("The closest obstacle is situated at " + str(round(obstacleDistance, 2)) + " meters on the " + obstaclePosition + " of the drone.")


##
# @brief This function takes all data and writes it in the log file.
def log_feedback():
    global dronePosition, droneAttitudeDeg, droneLinVel, droneAngVelDeg
    global obstacleDistance, obstaclePosition
    
    # Create the list of all data and write it in the log file
    logDronePose = [str(dronePosition.x), str(dronePosition.y), str(dronePosition.z), str(droneAttitudeDeg[0]), str(droneAttitudeDeg[1]), str(droneAttitudeDeg[2])]
    logDroneVelocities = [str(droneLinVel.x), str(droneLinVel.y), str(droneLinVel.z), str(droneAngVelDeg.x), str(droneAngVelDeg.y), str(droneAngVelDeg.z)]
    logObstacleInfo = [str(obstacleDistance), obstaclePosition]
    data = logDronePose + logDroneVelocities + logObstacleInfo
    outputCsv.writerow(data)
    csvFile.flush()


##
# @brief The function that shows the output on the terminal and logs data.
# 
# The function waits until both the Odometry and ClosestObstacleInfo messages
# are received, and then calls the log_feedback and print_feedback functions.
def feedback():
    
    # Show feedback every second
    rate = rospy.Rate(1)
    
    while not rospy.is_shutdown():
        # Clear the terminal
        os.system("clear")
    
        # Wait until both messages have been received
        odometryReceived.wait()
        infoReceived.wait()
    
        # Log the feedback on a csv file
        log_feedback()
        
        # Print the feedback on the terminal
        print_feedback()
        
        # Clear the flags
        odometryReceived.clear()
        infoReceived.clear()
        
        rate.sleep()
        
    
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
		
		# Open the log file
        csv_path = rospy.get_param("output_csv_path")
        csvFile = open(csv_path, 'w')
        outputCsv = csv.writer(csvFile, delimiter=',', quotechar='"')
        # Write the header that defines the contents of the log file
        header = ["drone_pos_x", "drone_pos_y", "drone_pos_z", "drone_roll", "drone_pitch", "drone_yaw", "drone_lin_vel_x", "drone_lin_vel_y", "drone_lin_vel_z", "drone_ang_vel_x", "drone_ang_vel_y", "drone_ang_vel_z", "closest_obs_distance", "closest_obs_pos"]
        outputCsv.writerow(header)
        csvFile.flush()
        
        # Start showing feedback
        feedback()
        
    except rospy.ROSInterruptException:
        pass
