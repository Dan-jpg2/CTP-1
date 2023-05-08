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
import time
import smbus2
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

LINEAR_VEL = 0.25
STOP_DISTANCE = 0.32
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
FRONT_STOP_DIST = 0.3 + LIDAR_ERROR



class RGBsensor(): #RGB sensor class
    def __init__(self): #RGB sensor initialization
        self.bus = smbus2.SMBus(1) #I2C bus 1 is used for the ISL29125 RGB sensor 
        time.sleep(0.5) #Wait for the initialization to finish 
        self.bus.write_byte_data(0x44, 0x01 , 0x05) # Configure the RGB Sensor's register to read RGB values
    
    #Reads the RGB sensor data and returns the values as a tuple (Red, Green, Blue)
    def get_color(self):
        # Read data from I2C interface
        Green = self.bus.read_word_data(0x44, 0x09)
        Red = self.bus.read_word_data(0x44, 0x0B)
        Blue= self.bus.read_word_data(0x44, 0x0D)
        return (Red, Green, Blue)

class Obstacle():
    def __init__(self):
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1) # Try queue size 0??
        self.remaining_angle = 0 # the remaining angle to turn
        self._turn_speed = 0.5 # the turning speed
        self.accumulated_speed = 0
        self.speed_updates = 0
        self.average_speed = 0
        self.obstacle() # call the obstacle method
    

    def get_scan(self):
        scan = rospy.wait_for_message('scan', LaserScan) # wait for laser scan message
        scan_filter = [] # create empty array to store filtered lidar data
        # https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html    

        for i in range(len(scan.ranges)): # iterate through the lidar samples
            scan_filter.append(scan.ranges[i]) # append the filtered lidar data to the list

        for i in range(len(scan_filter)): # iterate through the filtered lidar data
            curRead = scan_filter[i]
            if curRead == float('Inf') or curRead < 0.01 or math.isnan(curRead): # check if the lidar data is gibberish
                scan_filter[i] = 3.5 # set the lidar data to a large number (invalid)
        return scan_filter


    def obstacle(self):
        twist = Twist() # create a Twist message to send velocity commands
        turtlebot_moving = True # boolean to indicate whether the robot is moving or not
        RGB_cd = 0 # RGB cooldown
        victims = 0 # number of victims found
        collision_count = 0 # number of collisions
        collision_cd = 0 # collision cooldown
        sensor = RGBsensor() # initialize RGB sensor
        timeToRun = 60 * 2 # 2 minutes run time
        endTime = time.time() + timeToRun

        while (not rospy.is_shutdown() and (time.time() < endTime)): # loop until user presses Ctrl+C
            scan_read = self.get_scan() # get the filtered lidar data
            samples = len(scan_read)

            # Discard readings beyond the scope we want to work with here (take from -120 to +120 deg)
            lidar_distances = scan_read[2*int(samples/3):] 
            lidar_distances.extend(scan_read[:int(samples/3)])

            samples = len(lidar_distances) #update samples
            #Partition readings into cones for evaluating navigation from
            frontCone = lidar_distances[int(samples/3):int(-samples/3)] # -40 deg to +40 deg
            rightCone = lidar_distances[int(-samples/3):] # 40 deg to 120 deg
            leftCone = lidar_distances[:int(samples/3)] # -120 deg to -40 deg
            min_distance = min(lidar_distances) # get the minimum distance of the filtered lidar data
            rospy.loginfo('Lidar min dist: %f', min_distance)
            #rospy.loginfo('Array length %d', samples)
            
            

            if min(frontCone) < FRONT_STOP_DIST:
                if collision_cd < 1:
                    collision_count += 1
                    collision_cd = 5
                    rospy.loginfo('Collision detected, total collisions: %d', collision_count)
                    collision_cd -= 1 # Cooldown for loop cycles on colissions

                partition = int(len(frontCone)/2) # Use to look more detailed in the cones
                rightEval1 = min(rightCone[:partition])
                rightEval2 = min(rightCone[partition:])
                leftEval1 = min(leftCone[partition:])
                leftEval2 = min(leftCone[:partition])

                
                if rightEval1 < SAFE_STOP_DISTANCE:
                    twist.angular.z = -0.9 * 1/(2*rightEval1)
                    #twist.linear.x = 0.08
                    rospy.loginfo('Left!')
                elif rightEval2 < SAFE_STOP_DISTANCE:
                    twist.angular.z = -0.45 * 1/(3*rightEval2)
                    #twist.linear.x = 0.1
                    rospy.loginfo('Slight left!')
                elif leftEval1 < SAFE_STOP_DISTANCE:
                    twist.angular.z = 0.9 * 1/(2*leftEval1)
                    #twist.linear.x = 0.08
                    rospy.loginfo('Right!')
                elif leftEval2 < SAFE_STOP_DISTANCE:
                    twist.angular.z = 0.45 * 1/(3*leftEval2)
                    #twist.linear.x = 0.1
                    rospy.loginfo('Slight right!')
                else:
                    twist.angular.z = 0.0
                    twist.linear.x = -LINEAR_VEL/2

            else:
                twist.linear.x = LINEAR_VEL
                twist.angular.z = 0.0
            self._cmd_pub.publish(twist)
            #rospy.loginfo('Distance to the obstacle %f:', min_distance)
            #Average linear speed here _________________________________________________________________
            self.accumulated_speed += abs(twist.linear.x)
            self.speed_updates += 1
            self.average_speed = self.accumulated_speed / self.speed_updates
            #self._cmd_pub.publish(twist)
        rospy.loginfo('Average speed: %f', self.average_speed)

def main():
    rospy.init_node('turtlebot3_obstacle')
    try:
        obstacle = Obstacle()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()