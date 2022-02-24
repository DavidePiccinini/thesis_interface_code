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
import datetime
import time
import os
import threading
import csv
import tf_conversions
from nav_msgs.msg import Odometry
from interface_code.msg import ClosestObstacleInfo
from aruco_msgs.msg import IdentifiedMarkersIDs
from colorama import Fore, Back, Style

## Ratio to convert rad to deg
radToDegRatio = 180 / math.pi

## Threading event
odometryReceived = threading.Event()
## Threading event
infoReceived = threading.Event()
## Threading event
idsReceived = threading.Event()

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

## Marker IDs variable
markerIDs = []

## Time variable
startingTime = None
## Time variable
currentTime = None


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
# @brief The callback function for the @c '/aruco_marker_publisher/identified_marker_ids'
#        topic.
# 
# @param idsMsg The int32[] message.
def marker_ids_callback(idsMsg):
    global markerIDs
    
    # Process the message only when the previous one has been logged
    if not idsReceived.is_set():
        if idsMsg.ids:
            for id in idsMsg.ids:
                if id not in markerIDs:
                   markerIDs.append(id)
        
        # Set the flag to true
        idsReceived.set()


##
# @brief This function simply prints the important information about the
#        drone state in an user-friendly way.
def print_feedback():
    global dronePosition, droneAttitudeDeg, droneLinVel, droneAngVelDeg
    global obstacleDistance, obstaclePosition
    global markerIDs
    global startingTime, currentTime
    
    # Threshold in meters after which the terminal warns the pilot
    distanceThreshold = 1.5
    
    # Compute the time elapsed from the start of the experiment
    currentTime = int(time.time())
    timePeriod = currentTime - startingTime
    minutes = int(math.floor(timePeriod / 60))
    seconds = timePeriod % 60
    
    # Print on the terminal the information that is useful to the pilot
    print(Style.BRIGHT + Fore.YELLOW + "Current elapsed time from the start of the experiment:" + Style.RESET_ALL)
    print(Style.BRIGHT + Fore.YELLOW + "* " + str(minutes) + " minutes and " + str(seconds) + " seconds." + Style.RESET_ALL)
    print
    print("The current position (in meters) of the drone is:")
    print(Style.BRIGHT + "* x: " + str(round(dronePosition.x, 2)) + "\n* y: " + str(round(dronePosition.y, 2)) + "\n* z: " + str(round(dronePosition.z, 2)) + Style.RESET_ALL)
    print
    print("The current yaw attitude (in degrees) of the drone is:")
    print(Style.BRIGHT + "* Yaw: " + str(round(droneAttitudeDeg[2], 2)) + Style.RESET_ALL)
    print
    print("The current linear velocity (in m/s) of the drone is:")
    print(Style.BRIGHT + "* Velocity in x: " + str(round(droneLinVel.x, 2)) + "\n* Velocity in y: " + str(round(droneLinVel.y, 2)) + "\n* Velocity in z: " + str(round(droneLinVel.z, 2)) + Style.RESET_ALL)
    print
    print("The current yaw angular velocity (in deg/s) of the drone is:")
    print(Style.BRIGHT + "* Rotation about the Z axis: " + str(round(droneAngVelDeg.z, 2)) + Style.RESET_ALL)
    print
    
    if obstaclePosition == "none":
        print(Style.BRIGHT + Fore.YELLOW + "There are no detected obstacles at the moment." + Style.RESET_ALL)
    elif obstacleDistance <= distanceThreshold:
        print(Style.BRIGHT + Fore.RED + "The closest obstacle is situated at " + str(round(obstacleDistance, 2)) + " meters on the " + obstaclePosition + " of the drone." + Style.RESET_ALL)
    else:
        print(Style.BRIGHT + Fore.GREEN + "The closest obstacle is situated at " + str(round(obstacleDistance, 2)) + " meters on the " + obstaclePosition + " of the drone." + Style.RESET_ALL)
    print
    
    if not markerIDs:
        print(Style.BRIGHT + Fore.YELLOW + "There are no identified markers at the moment." + Style.RESET_ALL)
    else:
        print(Style.BRIGHT + Fore.GREEN + "The markers with the following IDs have been identified: " + str(markerIDs) + Style.RESET_ALL)


##
# @brief This function takes all data and writes it in the log file.
def log_feedback():
    global dronePosition, droneAttitudeDeg, droneLinVel, droneAngVelDeg
    global obstacleDistance, obstaclePosition
    global markerIDs
    
    # Create the list of all data and write it in the log file
    currentTimestamp = [int(time.time())]
    logDronePose = [str(dronePosition.x), str(dronePosition.y), str(dronePosition.z), str(droneAttitudeDeg[0]), str(droneAttitudeDeg[1]), str(droneAttitudeDeg[2])]
    logDroneVelocities = [str(droneLinVel.x), str(droneLinVel.y), str(droneLinVel.z), str(droneAngVelDeg.x), str(droneAngVelDeg.y), str(droneAngVelDeg.z)]
    logObstacleInfo = [str(obstacleDistance), obstaclePosition]
    
    data = currentTimestamp + logDronePose + logDroneVelocities + logObstacleInfo + [markerIDs]
    outputCsv.writerow(data)
    csvFile.flush()


##
# @brief The function that shows the output on the terminal and logs data.
# 
# The function waits until the Odometry, ClosestObstacleInfo, and
# IdentifiedMarkersIDs messages are received, and then calls the log_feedback
# and print_feedback functions.
def feedback():
    
    # Show feedback every second
    rate = rospy.Rate(1)
    
    while not rospy.is_shutdown():
        # Clear the terminal
        os.system("clear")
    
        # Wait until all messages have been received
        odometryReceived.wait()
        infoReceived.wait()
        idsReceived.wait()
    
        # Log the feedback on a csv file
        log_feedback()
        
        # Print the feedback on the terminal
        print_feedback()
        
        # Clear the flags
        odometryReceived.clear()
        infoReceived.clear()
        idsReceived.clear()
        
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
        
        # Subscribe to the "/aruco_marker_publisher/identified_marker_ids" topic
        rospy.Subscriber('/aruco_marker_publisher/identified_marker_ids', IdentifiedMarkersIDs, marker_ids_callback)
		
		# Open the log file
        csv_path = rospy.get_param("output_csv_path")
        currentDate = datetime.datetime.now()
        path = csv_path + str(currentDate.year) + "-" + str(currentDate.month) + "-" + str(currentDate.day) + "-" + str(currentDate.hour) + "-" + str(currentDate.minute) + ".csv"
        csvFile = open(path, 'w')
        outputCsv = csv.writer(csvFile, delimiter=',', quotechar='"')
        # Write the header that defines the contents of the log file
        header = ["timestamp", "drone_pos_x", "drone_pos_y", "drone_pos_z", "drone_roll", "drone_pitch", "drone_yaw", "drone_lin_vel_x", "drone_lin_vel_y", "drone_lin_vel_z", "drone_ang_vel_x", "drone_ang_vel_y", "drone_ang_vel_z", "closest_obs_distance", "closest_obs_pos", "identified_markers_ids"]
        outputCsv.writerow(header)
        csvFile.flush()
        
        # Get the starting timestamp
        startingTime = int(time.time())
        
        # Start showing feedback
        feedback()
        
    except rospy.ROSInterruptException:
        pass
