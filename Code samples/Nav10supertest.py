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

import rospy
import time 
import math 
import smbus2
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import random

LINEAR_VEL = 0.22
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05 
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
EMERGENCY_STOP = 0.15
MAX_ANGLE = 1.0
DEFAULT_ANGLE = 3.

class RGBsensor(): #RGB sensor class
    def __init__(self): #RGB sensor initialization
        self.bus = smbus2.SMBus(1) #I2C bus 1 is used for the ISL29125 RGB sensor 
        time.sleep(0.5) #Wait for the initialization to finish 
        self.bus.write_byte_data(0x44, 0x01 , 0x15) # Configure the RGB Sensor's register to read RGB values
    
    #Reads the RGB sensor data and returns the values as a tuple (Red, Green, Blue)
    def get_rgb(self):
        # Read data from I2C interface
        # We get 12 meaningful bits in cur config
        # 12 bit res means max val of 4095
        green = self.bus.read_word_data(0x44, 0x09)
        red = self.bus.read_word_data(0x44, 0x0B)
        blue= self.bus.read_word_data(0x44, 0x0D) 
        return red, green, blue
    
class Obstacle():
    def __init__(self):
        self._cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.obstacle()
        
        self._turn_speed = 0.5
        self.accumulated_speed = 0
        self.speed_updates = 0
        self.average_speed = 0
    
    def get_scan(self):
        scan = rospy.wait_for_message('scan', LaserScan)
        scan_filter = []
        
        
        
        left_lidar_samples_range = 60
        right_lidar_samples_range = 345
        front_lidar_samples_range = 15
        # The lidar samples are filtered based on the left_angle and right_angle variables
        front_lidar_samples1    = scan.ranges[0:front_lidar_samples_range] # 0 to 15
        front_lidar_samples2    = scan.ranges[right_lidar_samples_range:360] # 345 to 360
        left_lidar_samples      = scan.ranges[front_lidar_samples_range:left_lidar_samples_range] # 15 to 60
        right_lidar_samples     = scan.ranges[300:right_lidar_samples_range] # 300 to 345
        front_left_lidar_samples = scan.ranges[14:22] # 14 to 22 
        front_right_lidar_samples = scan.ranges[338:346] # 338 to 346 
        
        scan_filter.append(left_lidar_samples) 
        scan_filter.append(front_lidar_samples1)
        scan_filter.append(front_lidar_samples2)
        scan_filter.append(right_lidar_samples)
        scan_filter.append(front_left_lidar_samples)
        scan_filter.append(front_right_lidar_samples)
        
        return scan_filter
    
    def obstacle(self):
        twist = Twist()
        turtlebot_moving = True
        
        RGB_CD = 0
        victims = 0
        collision_count = 0
        collision_cd = 0
        sensor = RGBsensor()
        timeToRun = 60 * 2
        endTime = time.time() + timeToRun
        curBlue = (sensor.get_rgb())[2]
        
        
        def small_turns(direction, angle, velocity):
            if(direction == 'left'):
                twist.linear.x = velocity
                twist.angular.z = angle
                self._cmd_pub.publish(twist)
                rospy.loginfo("Turning left")
                turtlebot_moving = False
            
            elif (direction == 'right'):
                twist.linear.x = velocity
                twist.angular.z = -angle
                self._cmd_pub.publish(twist)
                rospy.loginfo("Turning right")
                turtlebot_moving = False
                
        def emergency_test(l):
            #Calculation of the slices / cones 
            left_dist = np.mean(l[0])
            front_dist = np.mean(l[1] + l[2])
            right_dist = np.mean(l[3])
            front_left_dist = np.mean(l[4])
            front_right_dist = np.mean(l[5])
            
            #Calculation of special cases 
            sp_case_right = np.mean(l[3][27:35])
            sp_case_left = np.mean(l[0][10:18])
            
            
            if (front_dist < EMERGENCY_STOP and left_dist < EMERGENCY_STOP):
                small_turns('right', MAX_ANGLE, 0.0)
                
            elif (front_dist < EMERGENCY_STOP and right_dist < EMERGENCY_STOP):
                small_turns('left', MAX_ANGLE, 0.0)
                
            elif (front_dist < EMERGENCY_STOP):
                if (left_dist < right_dist):
                    small_turns('right', MAX_ANGLE, 0.0)
                else:
                    small_turns('left', MAX_ANGLE, 0.0)
            
            elif (sp_case_right < SAFE_STOP_DISTANCE and sp_case_left < SAFE_STOP_DISTANCE):
                make_180_turn()
                
            elif (front_left_dist < SAFE_STOP_DISTANCE):
                small_turns('right', MAX_ANGLE, 0.0)
            
            elif (front_right_dist < SAFE_STOP_DISTANCE):
                small_turns('left', MAX_ANGLE, 0.0)
            
        def make_180_turn(): #Turns 180 degrees to the left
            twist.linear.x = 0.0
            twist.angular.z = MAX_ANGLE
            self._cmd_pub.publish(twist)
            rospy.loginfo('Turning 180 degrees')
            time.sleep(1.11) #Might change this to 1.105634 - found that on interwebs
            turtlebot_moving = False
        
        def non_zero(l): #Convert list of tuples to a list of lists and makes the zeroes to 3.5
            #We do this because the lidar sensor sometimes gives 0.0 as a value and we can't use that 
            #List converting
            for i in range(len(l)):
                l[i] = list(l[i])
            #If list element is 0.0, change it to 3.5
            for i in range(len(l)):
                for j in range(len(l[i])):
                    if l[i][j] == 0.0:
                        l[i][j] = 3.5
        
        #Main loop from here
        while (not rospy.is_shutdown() and (time.time() < endTime)):
            
            newRed, dummy, newBlue = sensor.get_rgb()
            rospy.loginfo('Red value %d, Blue value %d', newRed, newBlue)
            if newBlue < curBlue * 0.8 or newBlue > curBlue * 1.2:
                curBlue = newBlue # new baseline
                if newRed > 400 and newBlue < 200: # Check if we are currently over a tag
                    victims += 1
                    rospy.loginfo('Victim found, total count: %d', victims)
            
            if min(front_dist) < 0.04 + LIDAR_ERROR or min(right_dist) < 0.05 + LIDAR_ERROR or min(left_dist) < 0.05 + LIDAR_ERROR:
                if collision_cd < 1:
                   collision_count += 1
                   collision_cd = 5
                   rospy.loginfo('Collision detected, total collisions: %d', collision_count)   
            collision_cd -= 1 # Cooldown for loop cycles on colissions
            
            lidar_distances = self.get_scan()
            non_zero(lidar_distances)
            #Calculation of the slices / cones
            left_dist = np.mean(lidar_distances[0])
            front_dist = np.mean(lidar_distances[1] + lidar_distances[2])
            right_dist = np.mean(lidar_distances[3])
            front_left_dist = np.mean(lidar_distances[4])
            front_right_dist = np.mean(lidar_distances[5])
            
            emergency_test(lidar_distances)
            
            #Here our different cases we might encounter
            #case 1 - obstacle is in front of the robot
            if (front_dist < SAFE_STOP_DISTANCE+0.08):
                print('obstacle in front - case 1')
                #We need to look ahead 
                if turtlebot_moving:
                    if (right_dist < left_dist):
                        while(front_dist < SAFE_STOP_DISTANCE + 0.08):
                            small_turns('left', 1.44*front_dist**(-0.29), 2.64*front_dist**2.42)
                            new_front = self.get_scan()
                            non_zero(new_front)
                            emergency_test(new_front)
                            front_dist = np.mean(new_front[1] + new_front[2])
                            
                    elif (left_dist < right_dist):
                        while (front_dist < SAFE_STOP_DISTANCE + 0.08):
                            small_turns('right', 1.44*front_dist**(-0.29), 2.64*front_dist**2.42)
                            new_front = self.get_scan()
                            non_zero(new_front)
                            emergency_test(new_front)
                            front_dist = np.mean(new_front[1] + new_front[2])
                
            #Case 2 - obstacle is to the left or the right of the robot
            elif(left_dist < SAFE_STOP_DISTANCE or right_dist < SAFE_STOP_DISTANCE):
                if turtlebot_moving:
                    if (front_right_dist < front_left_dist):
                        small_turns('left', DEFAULT_ANGLE, 0.1)
                    else: 
                        small_turns('right', DEFAULT_ANGLE, 0.1)
            #Case 3 - obstacle is in blind spot??
            elif (front_left_dist < SAFE_STOP_DISTANCE + 0.15 or front_right_dist < SAFE_STOP_DISTANCE+0.15):
                if turtlebot_moving:
                    if (front_right_dist < front_left_dist):
                        print('possible blind spot?')
                        while(front_right_dist < SAFE_STOP_DISTANCE + 0.08):
                            small_turns('left', 0.34*front_right_dist**(-0.93), 1.7*front_right_dist**2.23)
                            new_front_right = self.get_scan()
                            non_zero(new_front_right)
                            emergency_test(new_front_right)
                            front_right_dist = np.mean(new_front_right[5])
                        
                    elif (front_left_dist < front_right_dist):
                        print('possible blind spot?')
                        while(front_left_dist < SAFE_STOP_DISTANCE + 0.08):
                            small_turns('right', 0.34*front_left_dist**(-0.93), 1.7*front_left_dist**2.23)
                            new_front_left = self.get_scan()
                            non_zero(new_front_left)
                            emergency_test(new_front_left)
                            front_left_dist = np.mean(new_front_left[4])
            
            #Case 4 - no obstacles so go forward
            else:
                twist.linear.x = LINEAR_VEL
                twist.angular.z = 0.0
                turtlebot_moving = True
                self._cmd_pub.publish(twist)
            
            self.accumulated_speed += abs(twist.linear.x)
            self.speed_updates += 1
            self.average_speed = self.accumulated_speed / self.speed_updates
            
            
        
def main():
    rospy.init_node('turtlebot3_obstacle')
    try:
        obstacle = Obstacle()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()