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
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

LINEAR_VEL = 0.22
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR

accumulated_speed = 0
speed_updates = 0

#RGB sensor section (week 3)
import smbus2

class RGBsensor(): #RGB sensor class
    def __init__(self): #RGB sensor initialization
        self.bus = smbus2.SMBus(1) #I2C bus 1 is used for the Raspberry Pi 3 
        time.sleep(0.5) #Wait for the initialization to finish 

        self.bus.write_byte_data(0x44, 0x01 , 0x05) #Sets the integration time to 50ms 
    
    #Reads the RGB sensor data and returns the values as a tuple (Red, Green, Blue)
    def get_color(self):
        # Read data from I2C interface (the registers)
        data = self.bus.read_i2c_block_data(0x44, 0x09, 6)

        Green = data[1] 
        Blue = data[3]
        Red = data[5]
        return (data[1], data[3], data[5])  
    

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
        left_angle_index = int((math.radians(-45) - angle_min) / angle_inc)
        # get the index of the leftmost lidar sample
        right_angle_index = int((math.radians(45) - angle_min) / angle_inc)
        # get the index of the rightmost lidar sample

        for i in range(left_angle_index, right_angle_index + 1): # iterate through the lidar samples
            scan_filter.append(scan.ranges[i]) # append the filtered lidar data to the list

        for i in range(len(scan_filter)): # iterate through the filtered lidar data
            if scan_filter[i] == float('Inf'): # check if the lidar data is infinite
                scan_filter[i] = 3.5 # set the lidar data to a large number
            elif math.isnan(scan_filter[i]): # check if the lidar data is NaN
                scan_filter[i] = 0 # set the lidar data to 0

        return scan_filter
    
    def check_collision(self):
        full_distance = self.get_scan() # get the filtered lidar data 
        if (min(full_distance) < SAFE_STOP_DISTANCE): # check if the minimum distance is less than the safe stop distance
            return True
        else:
            return False 
    
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

        # Lightsensor and collision detection loop 
        while(not rospy.is_shutdown()) and (time.time() < time_left):
            lightdata = sensor.get_color()

            if(self.check_collision() and collision_cd < 1):
                collision_count += 1
                collision_cd = 5
                rospy.loginfo("Collision detected, total collisions: {}".format(collision_count))
            collision_cd -= 1
            
            # RGB detection of victims of color red
            if RGB_cd <= 0:
                if lightdata[0] > 80:
                    victims += 1
                    RGB_cd = 5
                    rospy.loginfo("Victim found, total victims: {} ".format(victims))
            else:
                RGB_cd -= 1

        while not rospy.is_shutdown(): # loop until user presses Ctrl+C
            lidar_distances = self.get_scan() # get the filtered lidar data
            min_distance = min(lidar_distances) # get the minimum distance of the filtered lidar data

            if min_distance < SAFE_STOP_DISTANCE: # check if the minimum distance is less than the safe stop distance
                if turtlebot_moving: 
                    twist.linear.x = 0.0 
                    twist.angular.z = 0.0
                    self._cmd_pub.publish(twist)
                    turtlebot_moving = False
                    rospy.loginfo('Stop!')

                    # Calculate the turn angle
                    left_min_distance = min(lidar_distances[:len(lidar_distances)//2]) # get the minimum distance of the left side of the robot
                    right_min_distance = min(lidar_distances[len(lidar_distances)//2:]) # get the minimum distance of the right side of the robot
                    if left_min_distance < right_min_distance: # check which side has the minimum distance
                        angle = math.atan((SAFE_STOP_DISTANCE - left_min_distance) / LIDAR_ERROR) # calculate the turn angle
                        self._remaining_angle = angle / 2 # set the remaining angle to turn
                        twist.angular.z = self._turn_speed # set the angular velocity
                        rospy.loginfo('Turn right {} degrees!'.format(angle)) # log the turn angle

                    elif right_min_distance < left_min_distance: # check which side has the minimum distance
                        angle = math.atan((SAFE_STOP_DISTANCE - right_min_distance) / LIDAR_ERROR) # calculate the turn angle
                        self._remaining_angle = angle / 2 # set the remaining angle to turn
                        twist.angular.z = -self._turn_speed # set the angular velocity
                        rospy.loginfo('Turn left {} degrees!'.format(angle)) # log the turn angle
                    else:
                        twist.linear.x = -LINEAR_VEL # set the linear velocity
                        rospy.loginfo('Go backward!') # log the linear velocity

                else:
                    # Check if there's a new obstacle that requires a new turn
                    if self._remaining_angle > 0: # check if the robot has finished turning
                        self._remaining_angle -= abs(twist.angular.z) * (1.0/20) # calculate the remaining angle to turn
                        twist.angular.z = self._turn_speed if twist.angular.z > 0 else -self._turn_speed # set the angular velocity
                    else:
                        turtlebot_moving = True
                        twist.linear.x = LINEAR_VEL
                        twist.angular.z = 0.0
                        rospy.loginfo('Go forward!')

            else:
                turtlebot_moving = True
                twist.linear.x = LINEAR_VEL
                twist.angular.z = 0.0
                self._remaining_angle = 0
                rospy.loginfo('Go forward!')

            self._cmd_pub.publish(twist)

        #Average linear speed here _________________________________________________________________
        accumulated_speed += abs(twist.linear.x)
        speed_updates += 1
        average_speed = accumulated_speed / speed_updates
        self._cmd_pub.publish(twist)



        rospy.loginfo('Average linear speed: {}'.format(average_speed))
        

def main():
    rospy.init_node('turtlebot3_obstacle')
    try:
        obstacle = Obstacle()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()