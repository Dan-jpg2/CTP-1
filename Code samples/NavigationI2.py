#!/usr/bin/env python
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################
#
# Authors: Gilbert #
#
#Explanation of what is changed:
#
#I added comments explaining the new samples_view, left_angle, and right_angle variables, which determine the number of lidar samples
#and the range of angles to use for obstacle detection. 
#
#I also added comments to the get_scan() method to explain how the lidar samples are filtered based on these parameters. 
#Additionally, I removed the unnecessary while loop in the main() function and added a __name__ check to allow 
#the script to be imported as a module.
#
import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

LINEAR_VEL = 0.22
STOP_DISTANCE = 0.35
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
FRONT_STOP_DIST = 0.35 + LIDAR_ERROR

class Obstacle():
    def __init__(self):
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.obstacle()
        
    def get_scan(self):
        scan = rospy.wait_for_message('scan', LaserScan) # wait for laser scan message
        scan_filter = [] # create empty list to store filtered lidar data
   
        samples = len(scan.ranges) # get the number of lidar samples
        angle_min = scan.angle_min # get the minimum angle of the lidar
        angle_max = scan.angle_max # get the maximum angle of the lidar
        angle_inc = scan.angle_increment # get the angle increment of the lidar

        # The number of lidar samples to use for obstacle detection
        left_angle_index = int((math.radians(-90) - angle_min) / angle_inc)
        # get the index of the leftmost lidar sample
        right_angle_index = int((math.radians(90) - angle_min) / angle_inc)
        # get the index of the rightmost lidar sample

        for i in range(left_angle_index, right_angle_index + 1): # iterate through the lidar samples
            scan_filter.append(scan.ranges[i]) # append the filtered lidar data to the list

        for i in range(len(scan_filter)): # iterate through the filtered lidar data
            if scan_filter[i] == float('Inf'): # check if the lidar data is infinite
                scan_filter[i] = 3.5 # set the lidar data to a large number
            elif math.isnan(scan_filter[i]): # check if the lidar data is NaN
                scan_filter[i] = 100 # set the lidar data to 100 (invalid)

        return scan_filter
    
    def obstacle(self):
        twist = Twist()
        turtlebot_moving = True

        while not rospy.is_shutdown(): # loop until user presses Ctrl+C
            lidar_distances = self.get_scan() # get the filtered lidar data
            samples = len(lidar_distances)
            for i in range(0, samples):
                if lidar_distances[i] == 0:
                    lidar_distances[i] = 100
            samples = len(lidar_distances)
            rospy.loginfo('Array length %d', samples)
            frontCone = lidar_distances[samples/3:-samples/3]
            rightCone = lidar_distances[:-samples/3]
            leftCone = lidar_distances[:samples/3]
            min_distance = min(lidar_distances) # get the minimum distance of the filtered lidar data
            rospy.loginfo('Lidar min dist: %d', min_distance)
            
            if min(frontCone) < FRONT_STOP_DIST:
                rightPart = len(rightCone)/2
                leftPart = len(leftCone)/2
                if min(rightCone[:-rightPart]) < SAFE_STOP_DISTANCE:
                    twist.angular.z = -0.4
                    twist.linear.x = 0.08
                    rospy.loginfo('Left!')
                elif min(rightCone[:rightPart]) < SAFE_STOP_DISTANCE:
                    twist.angular.z = -0.02
                    twist.linear.x = 0.1
                    rospy.loginfo('Slight left!')
                elif min(leftCone[:-leftPart]) < SAFE_STOP_DISTANCE:
                    twist.angular.z = 0.4
                    twist.linear.x = 0.08
                    rospy.loginfo('Right!')
                elif min(leftCone[:leftPart]) < SAFE_STOP_DISTANCE:
                    twist.angular.z = 0.02
                    twist.linear.x = 0.1
                    rospy.loginfo('Slight right!')s
            else:
                twist.linear.x = LINEAR_VEL
                twist.angular.z = 0.0
            self._cmd_pub.publish(twist)
            rospy.loginfo('Distance to the obstacle %f:', min_distance)

def main():
    rospy.init_node('turtlebot3_obstacle')
    try:
        obstacle = Obstacle()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()