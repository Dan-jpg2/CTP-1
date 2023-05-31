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

LINEAR_VEL = 0.22 # Corrected for reverse motors on this robot
LIDAR_ERROR = 0.05 # prev 0.05
SAFE_STOP_DISTANCE = 0.13 + LIDAR_ERROR
FRONT_SAFE_DIST = 0.27 + LIDAR_ERROR
EMERGENCY_STOP_DIST = 0.08 + LIDAR_ERROR


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
        blue = self.bus.read_word_data(0x44, 0x0D)
        return red, green, blue

class Obstacle():
    def __init__(self):
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=0)
        self.remaining_angle = 0 # the remaining angle to turn
        self.accumulated_speed = 0
        self.speed_updates = 0
        self.average_speed = 0
        self.obstacle() # Done initializing, begin operation
    

    def get_scan(self):
        scan = rospy.wait_for_message('scan', LaserScan) # Listener for next 'scan' message from the lidar
        scan_filter = [] # create empty array to store filtered lidar data  

        for i in range(len(scan.ranges)): # iterate through the lidar samples
            scan_filter.append(scan.ranges[i]) # append the filtered lidar data to the list

        for i in range(len(scan_filter)): # iterate through the filtered lidar data
            curRead = scan_filter[i]
            if curRead == float('Inf') or curRead < 0.01 or math.isnan(curRead): # check if the lidar data is gibberish
                scan_filter[i] = 10.0 # set the lidar data to a large number (invalid)
        return scan_filter
    

    def obstacle(self):
        twist = Twist() # create a Twist message to send velocity commands
        
        victims = 0 # Counter for found victims
        collision_count = 0 # Counter for collisions
        collision_cd = 0 # Cooldown counter for collisions
        
        sensor = RGBsensor() # initialize RGB sensor
        curBlue = (sensor.get_rgb())[2] # Get first baseline reading for detecting victims
        
        cornered = False # Flag for getting out of corners (Default = False)
        rightTurn = True # Flag for which way to turn (Default = True)
        
        rightLoop = 0
        leftLoop = 0
        
        timeToRun = 60 * 2 # 2 minutes run time
        endTime = time.time() + timeToRun

        def driveUpdate(angular, linear): # Simplify updating twist message
            twist.angular.z = angular
            twist.linear.x = linear

        while (not rospy.is_shutdown() and (time.time() < endTime)): # loop for 2 minutes or until user CTRL + C
            scan_read = self.get_scan() # get the filtered lidar data
            
            # Discard readings beyond the scope we want to work with here (take from -90 to +90 deg)
            lidar_distances = scan_read[:90][::-1]
            lidar_distances.extend(scan_read[270:][::-1]) # Store our data from left to right view   

            #Partition readings into cones for evaluating navigation
            frontCone = lidar_distances[65:115] # -25 deg to +25 deg
            rightCone = lidar_distances[115:] # 30 deg to 90 deg
            leftCone = lidar_distances[:65] # -90 deg to -30 deg

            rightEval = min(rightCone)
            leftEval = min(leftCone)
            frontEval = min(frontCone)
            narrowFront = min(frontCone[15:45]) # Front cone of +-15 deg
            leftTurnE = min(lidar_distances[25:90]) # Eval for left side turn
            rightTurnE = min(lidar_distances[90:155]) # right turn eval

            newRed, dummy, newBlue = sensor.get_rgb() # Get values for victim detection (red and blue)
            
            if newBlue < curBlue * 0.75 or newBlue > curBlue * 1.25: # Only care about the RGB readings if the data is 20% different from last baseline (prevents double counts)
                curBlue = newBlue # new baseline
                if newRed > 400 and newBlue < 200: # Check if we are currently over a red tag
                    victims += 1
                    rospy.loginfo('Victim found, total count: %d', victims)
            
            if min(scan_read) < 0.05 + LIDAR_ERROR: # If something is within 5 cm (+ error) count up collision (unless on cooldown)
                if collision_cd < 1:
                   collision_count += 1
                   collision_cd = 10
                   rospy.loginfo('Collision detected, total collisions: %d', collision_count)   
            collision_cd -= 1 # Cooldown for loop cycles on colissions

            if(cornered): # If we've been cornered, this lets us turn until we find a way out
                leftLoop = 0
                rightLoop = 0
                if(frontEval > EMERGENCY_STOP_DIST * 1.6):
                    cornered = False
                    driveUpdate(0.0, LINEAR_VEL)
                else:
                    if(rightTurn):
                        driveUpdate(-1.0, 0.0)
                    else:
                        driveUpdate(1.0, 0.0)
            
            elif frontEval < FRONT_SAFE_DIST: # If obstacle in front, turn
                # Turn direction decision
                if(leftTurnE <= rightTurnE):
                    rightTurn = True
                else:
                    rightTurn = False

                if(narrowFront < EMERGENCY_STOP_DIST): # Determine if cornered
                    cornered = True
                    driveUpdate(0.0, -0.15)

                elif(rightTurn): # Need to turn, not cornered
                    #Turn right
                    driveUpdate(-0.7 - 0.16/leftEval, LINEAR_VEL) # Default turn a little if something is in the way
                    rightLoop += 1
                    leftLoop = 0
                    if(rightLoop > 10): # if we seem stuck on a corner we can't see, reverse
                        driveUpdate(1.4, -0.4 * LINEAR_VEL)
                    
                    elif(leftEval < SAFE_STOP_DISTANCE):
                        driveUpdate(-1.0 - 0.22/leftEval, LINEAR_VEL) # Turn depending on how close an obstacle is
                else:
                    #Turn left 
                    driveUpdate(0.7 + 0.16/rightEval, LINEAR_VEL) # prev 0.5 + 0.15
                    leftLoop += 1
                    rightLoop = 0
                    if(leftLoop > 10):
                        driveUpdate(-1.4, -0.4 * LINEAR_VEL)

                    elif(rightEval < SAFE_STOP_DISTANCE):
                        driveUpdate(1.0 + 0.22/rightEval, LINEAR_VEL)

            else: # Continue straight, if no obstacles ahead
                leftLoop = 0
                rightLoop = 0
                driveUpdate(0.0, LINEAR_VEL)
            self._cmd_pub.publish(twist) # Publish our decision on what to do
                
            #Average linear linear updates here
            self.accumulated_speed += abs(twist.linear.x)
            self.speed_updates += 1
            self.average_speed = self.accumulated_speed / self.speed_updates

        rospy.loginfo('Average linear speed: %f\nVictims Found: %d\nCollisions detected: %d', self.average_speed, victims, collision_count)
        # Print variables at the end of run
        
def main():
    rospy.init_node('turtlebot3_obstacle')
    try:
        obstacle = Obstacle()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()