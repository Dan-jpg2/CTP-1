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
SAFE_STOP_DISTANCE = 0.14 + LIDAR_ERROR
FRONT_SAFE_DIST = 0.26 + LIDAR_ERROR
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
        blue= self.bus.read_word_data(0x44, 0x0D) 
        return red, green, blue

class Obstacle():
    def __init__(self):
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1) # Try queue size 0??
        self.remaining_angle = 0 # the remaining angle to turn
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
                scan_filter[i] = 1.0 # set the lidar data to a large number (invalid)
        return scan_filter
    

    def obstacle(self):
        twist = Twist() # create a Twist message to send velocity commands
        RGB_cd = 0 # RGB cooldown
        victims = 0 # number of victims found
        collision_count = 0 # number of collisions
        collision_cd = 0 # collision cooldown
        sensor = RGBsensor() # initialize RGB sensor
        timeToRun = 60 * 2 # 2 minutes run time
        endTime = time.time() + timeToRun
        curBlue = (sensor.get_rgb())[2]
        stuck = False
        rightTurn = True # Default case, we have to define this before use

        def driveUpdate(dir, speed):
            twist.angular.z = dir
            twist.linear.x = speed

        while (not rospy.is_shutdown() and (time.time() < endTime)): # loop for 2 minutes or until user CTRL + C
            scan_read = self.get_scan() # get the filtered lidar data
            samples = len(scan_read)

            # Discard readings beyond the scope we want to work with here (take from -120 to +120 deg)
            lidar_distances = scan_read[2*int(samples/3):] 
            lidar_distances.extend(scan_read[:int(samples/3)])

            newRed, dummy, newBlue = sensor.get_rgb()
            rospy.loginfo('Red value %d, Blue value %d', newRed, newBlue)
            if newBlue < curBlue * 0.8 or newBlue > curBlue * 1.2:
                curBlue = newBlue # new baseline
                if newRed > 400 and newBlue < 200: # Check if we are currently over a red tag
                    victims += 1
                    rospy.loginfo('Victim found, total count: %d', victims)

            samples = len(lidar_distances) #update samples
            #Partition readings into cones for evaluating navigation from
            frontCone = lidar_distances[90:150] # -30 deg to +30 deg
            rightCone = lidar_distances[:90] # 30 deg to 120 deg
            leftCone = lidar_distances[150:] # -120 deg to -30 deg
            min_distance = min(lidar_distances) # get the minimum distance of the filtered lidar data
            rospy.loginfo('Lidar min dist: %f', min_distance)

            part = int(len(rightCone)/2) # Use to look more detailed in the side cones
            frontPart = int(len(frontCone)/2) # Split front cone in left and right
            frontRight = min(frontCone[:frontPart])
            frontLeft = min(frontCone[frontPart:])
            rightEval1 = min(rightCone[part:])
            #rightEval2 = min(rightCone[:part])
            leftEval1 = min(leftCone[:part])
            #leftEval2 = min(leftCone[part:])
            frontEval = min(frontCone)
            narrowFront = min(lidar_distances[105:135])
            rightEval = min(lidar_distances[:120])
            leftEval = min(lidar_distances[120:])

            print('Front right min:', frontRight)
            print('Front left min:', frontLeft)
            print('right eval min:', rightEval)
            print('left eval min:', leftEval)
            print('right1 eval min:', rightEval1)
            #print('right2 eval min:', rightEval2)
            print('left1 eval min:', leftEval1)            
            #print('left2 eval min:', leftEval2)

            if min(frontCone) < 0.04 or min(rightCone) < 0.05 or min(leftCone) < 0.05:
                if collision_cd < 1:
                   collision_count += 1
                   collision_cd = 5
                   rospy.loginfo('Collision detected, total collisions: %d', collision_count)   
            collision_cd -= 1 # Cooldown for loop cycles on colissions

            if(stuck):
                if(frontEval > EMERGENCY_STOP_DIST * 1.5):
                    stuck = False
                    driveUpdate(0.0, LINEAR_VEL)
                else:
                    rospy.loginfo('Stuck')
                    if(rightTurn):
                        driveUpdate(-1.0, 0.0)
                    else:
                        driveUpdate(1.0, 0.0)
            
            elif frontEval < FRONT_SAFE_DIST:
                # Turn decision
                if(leftEval <= rightEval):
                    rightTurn = True
                else:
                    rightTurn = False

                if(narrowFront < EMERGENCY_STOP_DIST): # Determine if stuck
                    stuck = True
                    driveUpdate(0.0, -0.1)

                elif(rightTurn): 
                    #Turn right
                    driveUpdate(-0.5 - 0.1/leftEval, LINEAR_VEL) # Default turn a little if somethign is in the way
                    if(leftEval1 < SAFE_STOP_DISTANCE):
                        driveUpdate(-1.0 - 0.2/leftEval1, LINEAR_VEL)
                        rospy.loginfo('Right!')
                else:
                    #Turn left
                    driveUpdate(0.5 + 0.1/rightEval, LINEAR_VEL) # Default small turn
                    if(rightEval1 < SAFE_STOP_DISTANCE):
                        driveUpdate(1.0 + 0.2/rightEval1, LINEAR_VEL)
                        rospy.loginfo('Left!')

            else:
                driveUpdate(0.0, LINEAR_VEL)
                rospy.loginfo('Continue straight')
            self._cmd_pub.publish(twist)
                
            #Average linear speed here 
            self.accumulated_speed += abs(twist.linear.x)
            self.speed_updates += 1
            self.average_speed = self.accumulated_speed / self.speed_updates

            time.sleep(0.2) # Sleep to delay evaluation for new data, get_scan doesn't work too fast
        rospy.loginfo('Average speed: %f\nVictims Found: %d\nCollisions detected: %d', self.average_speed, victims, collision_count)
        
def main():
    rospy.init_node('turtlebot3_obstacle')
    try:
        obstacle = Obstacle()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()