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

LINEAR_VEL = 0.22
STOP_DISTANCE = 0.35
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
FRONT_STOP_DIST = 0.35 + LIDAR_ERROR

accumulated_speed = 0
speed_updates = 0

class RGBsensor(): #RGB sensor class
    def __init__(self): #RGB sensor initialization
        self.bus = smbus2.SMBus(1) #I2C bus 1 is used for the Raspberry Pi 3 
        time.sleep(0.5) #Wait for the initialization to finish 

        self.bus.write_byte_data(0x44, 0x01 , 0x05) #Sets the integration time to 50ms 
    
    #Reads the RGB sensor data and returns the values as a tuple (Red, Green, Blue)
    def get_color(self):
        # Read data from I2C interface (the registers)
        Green = self.bus.read_word_data(0x44, 0x09)
        Red = self.bus.read_word_data(0x44, 0x0B)
        Blue= self.bus.read_word_data(0x44, 0x0D)
        return (Red, Green, Blue)

class Obstacle():
    def __init__(self):
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1) 
        self.remaining_angle = 0 # the remaining angle to turn
        self._turn_speed = 0.5 # the turning speed
        self.obstacle() # call the obstacle method
    

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
                scan_filter[i] = 10 # set the lidar data to 100 (invalid)

        return scan_filter
    


    def obstacle(self):
        twist = Twist() # create a Twist message to send velocity commands
        turtlebot_moving = True # boolean to indicate whether the robot is moving or not
        RGB_cd = 0 # RGB cooldown
        victims = 0 # number of victims found
        collision_count = 0 # number of collisions
        collision_cd = 0 # collision cooldown
        sensor = RGBsensor() # initialize RGB sensor
        time_to_run = 60 * 2 # 2 minutes
        time_left = time.time() + time_to_run

        while (not rospy.is_shutdown() and (time.time() < time_left)): # loop until user presses Ctrl+C
            lidar_distances = self.get_scan() # get the filtered lidar data
            samples = len(lidar_distances)
            for i in range(0, samples):
                if lidar_distances[i] == 0:
                    lidar_distances[i] = 10
            frontCone = lidar_distances[samples/3:-samples/3]
            rightCone = lidar_distances[:-samples/3]
            leftCone = lidar_distances[:samples/3]
            min_distance = min(lidar_distances) # get the minimum distance of the filtered lidar data
            rospy.loginfo('Lidar min dist: %d', min_distance)
            rospy.loginfo('Array length %d', samples)
            
            if collision_cd < 1:
                collision_count += 1
                collision_cd = 5
                rospy.loginfo("Collision detected, total collisions: {}".format(collision_count))
            collision_cd -= 1 # Cooldown for loop cycles on colissions

            if min(frontCone) < FRONT_STOP_DIST:
                rightPart = len(rightCone)/2
                leftPart = len(leftCone)/2
                if min(rightCone[:-rightPart]) < SAFE_STOP_DISTANCE:
                    twist.angular.z = -0.9
                    twist.linear.x = 0.08
                    rospy.loginfo('Left!')
                elif min(rightCone[:rightPart]) < SAFE_STOP_DISTANCE:
                    twist.angular.z = -0.45
                    twist.linear.x = 0.1
                    rospy.loginfo('Slight left!')
                elif min(leftCone[:-leftPart]) < SAFE_STOP_DISTANCE:
                    twist.angular.z = 0.9
                    twist.linear.x = 0.08
                    rospy.loginfo('Right!')
                elif min(leftCone[:leftPart]) < SAFE_STOP_DISTANCE:
                    twist.angular.z = 0.45
                    twist.linear.x = 0.1
                    rospy.loginfo('Slight right!')
            else:
                twist.linear.x = LINEAR_VEL
                twist.angular.z = 0.0
            self._cmd_pub.publish(twist)
            rospy.loginfo('Distance to the obstacle %f:', min_distance)
            #Average linear speed here _________________________________________________________________
            accumulated_speed += abs(twist.linear.x)
            speed_updates += 1
            average_speed = accumulated_speed / speed_updates
            self._cmd_pub.publish(twist)


def main():
    rospy.init_node('turtlebot3_obstacle')
    try:
        obstacle = Obstacle()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()