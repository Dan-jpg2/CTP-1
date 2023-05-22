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

LINEAR_VEL = 1
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05 # prev 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
SAFE_TURN_DISTANCE = SAFE_STOP_DISTANCE * 2
COL_DISTANCE = 0.1



class RGBsensor(): #RGB sensor class
    RGB_CD = 0 # RGB cooldown
    def __init__(self): #RGB sensor initialization
        self.bus = smbus2.SMBus(1) #I2C bus 1 is used for the ISL29125 RGB sensor 
        time.sleep(0.5) #Wait for the initialization to finish 
        self.bus.write_byte_data(0x44, 0x01 , 0x05) # Configure the RGB Sensor's register to read RGB values
    
    #Reads the RGB sensor data and returns the values as a tuple (Red, Green, Blue)
    def get_color(self):
        # Read data from I2C interface
        data = self.bus.read_i2c_block_data(0x44, 0x09, 6)
        green = data[1] #high byte
        red = data[3] #high byte
        blue = data[5] #high byte
        return (data[1], data[3], data[5])
    
    def get_victims(self):
        data = self.get_color()
        if RGBsensor.RGB_CD <= 0:
            if data[0] > 40:
                RGBsensor.RGB_CD = 10
                return True
            else:
                RGBsensor.RGB_CD -= 1
                return False


class Obstacle():
    colission_count = 0 # number of collisions
    colission_cd = 0 # collision cooldown

    def __init__(self):
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1) # Try queue size 0??
        self.remaining_angle = 0 # the remaining angle to turn
        self._turn_speed = 0.5 # the turning speed
        self.accumulated_speed = 0
        self.speed_updates = 0
        self.average_speed = 0
        self.obstacle() # call the obstacle method


       
    

    def get_scan(self, angle):
        scan = rospy.wait_for_message('scan', LaserScan) # get the lidar scan data
        scan_filter = [] # create an empty list to store the filtered lidar data
        samples = len(scan.ranges) # get the number of samples
        samples_view = angle  # the number of samples to use for obstacle detection

        #If angle is bigger or equal to 360 angle is set to max
        if samples_view >= 360:
            samples_view = samples
        
        #if only one value, we dont filter
        if samples_view is 1: 
            scan.append(scan.ranges[0])
        else:
            left_lidar_range_ranges = -(samples_view // 2 + samples_view %2) # the left angle of the lidar range
            right_lidar_range_ranges = samples_view // 2 # the right angle of the lidar range

            left_lidar_samples = scan.ranges[left_lidar_range_ranges:] # the left sample of the lidar range
            right_lidar_samples = scan.ranges[:right_lidar_range_ranges] # the right sample of the lidar range
            scan_filter.extend(left_lidar_samples + right_lidar_samples) # add the left and right lidar samples to the scan_filter list

        #filter the values so we get rid of the inf values and wrong scans
        for i in range(samples_view):
            if scan_filter[i] == 0.0:
                scan_filter[i] = 3.5
                null_count = null_count + 1
            if scan_filter[i] == float('Inf'):
                scan_filter[i] = 3.5
                null_count = null_count + 1
            elif math.isnan(scan_filter[i]):
                scan_filter[i] = 0.01
        return scan_filter
    
    def col_check(self):
        full_dist = self.get_scan(360)
        if(min(full_dist) < COL_DISTANCE and Obstacle.collision_cooldown < 1):
            Obstacle.collision_counter += 1
            Obstacle.collision_cooldown = 5
            rospy.loginfo('COLLISION, total collisions: %i', Obstacle.collision_counter)
        else:
            Obstacle.collision_cooldown -= 1

    
    def obstacle(self):
        twist = Twist() # create a Twist message to send velocity commands
        turtlebot_moving = True # set the initial state to moving
        victims_found = 0  # number of victims found

        TMR_CD = 0 #If stuck in a rotational loop, this will be used to break out of it

        #Lightsensor
        lightsensor = RGBsensor()

        #Runtime loop - we have to run for 2 minutes
        runTime = 60 * 2
        endTime = time.time + runTime

        #Average speed
        accumulated_speed = 0
        speed_updates = 0

        #Start of our loop
        while (not rospy.is_shutdown()) and (time.time() < endTime):
            #Check colissions 
            self.col_check()

            #check victims
            if (lightsensor.get_victims()):
                victims_found += 1
                rospy.loginfo('Victim found, total victims: %i', victims_found)    

            #Get a lidar scan on 180 degree
            lidar_distance = self.get_scan(180)

            #Divide the scan into subgroups for a wider view with a bit of overlap as cones
            super_front_cone = lidar_distance[80:100] # 20 degrees
            front_cone = lidar_distance[55:125] # 70 degrees
            left_cone = lidar_distance[0:67] # 67 degrees
            right_cone = lidar_distance[113:180] # 67 degrees

            #Moving forward and not having to turn
            if turtlebot_moving:
                minDistance = min(super_front_cone)

                #If we are too close to an obstacle
                if minDistance < SAFE_STOP_DISTANCE: # 
                    twist.linear.x = -LINEAR_VEL*0.2 # Move backwards
                    turtlebot_moving = False
                else:
                    if minDistance < SAFE_TURN_DISTANCE: # If we are too close to an obstacle
                        min_index = front_cone.index(min(front_cone)) # Get the index of the closest obstacle
                        if min_index < 35: #90 degrees
                            factor = 1
                        else:
                            factor = -1
                            twist.linear.x = LINEAR_VEL # Move forward
                            twist.angular.z = factor * self._turn_speed # Turn
                    else:
                        twist.linear.x = LINEAR_VEL # Move forward
                        twist.angular.z = 0.0# Turn

                    TMR_CD = 4 # Reset the timer
                    turtlebot_moving = True
            else:
                minDistance = min(lidar_distance) # Get the minimum distance from the lidar scan
                if min(super_front_cone) > SAFE_STOP_DISTANCE*2: # If we are far enough from the obstacle
                    twist.linear.x = LINEAR_VEL # Move forward
                    TMR_CD = 4
                    turtlebot_moving = True
                else:
                    if TMR_CD > 0:
                        min_index = lidar_distance.index(min(lidar_distance)) # Get the index of the closest obstacle
                        if min_index >= 90:
                            factor = -1
                        else:
                            factor = 1
                    TMR_CD -= 1

                    twist.linear.x = 0.0
                    twist.angular.z = factor 
                    turtlebot_moving = False
        
            time.sleep(0.27) # Sleep to delay evaluation for new data, get_scan doesn't work too fast
            #rospy.loginfo('Distance to the obstacle %f:', min_distance)
            #Average linear speed here _________________________________________________________________
            self.accumulated_speed += abs(twist.linear.x)
            self.speed_updates += 1
            self.average_speed = self.accumulated_speed / self.speed_updates
            #
        rospy.loginfo('Average speed: %f', self.average_speed)
        rospy.loginfo('Number of victims: %f', victims_found)
        rospy.loginfo('Number of collisions: %f', Obstacle.)
        

def main():
    rospy.init_node('turtlebot3_obstacle')
    try:
        obstacle = Obstacle()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()