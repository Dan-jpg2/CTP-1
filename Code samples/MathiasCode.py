#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

LINEAR_VEL = 0.22
STOP_DISTANCE = 0.25
LIDAR_ERROR = 0.2 # Prev 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR

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
        left_angle_index = int((math.radians(-25) - angle_min) / angle_inc)
        # get the index of the leftmost lidar sample
        right_angle_index = int((math.radians(25) - angle_min) / angle_inc)
        # get the index of the rightmost lidar sample

        for i in range(left_angle_index, right_angle_index + 1): # iterate through the lidar samples
            scan_filter.append(scan.ranges[i]) # append the filtered lidar data to the list

        for i in range(len(scan_filter)): # iterate through the filtered lidar data
            if scan_filter[i] == float('Inf'): # check if the lidar data is infinite
                scan_filter[i] = 3.5 # set the lidar data to a large number
            elif math.isnan(scan_filter[i]): # check if the lidar data is NaN
                scan_filter[i] = 0 # set the lidar data to 0

        return scan_filter

    def obstacle(self):
        twist = Twist()
        turtlebot_moving = True
        Obstacle_count = 0

        while not rospy.is_shutdown(): # loop until user presses Ctrl+C
            lidar_distances = self.get_scan() # get the filtered lidar data

            for i in range(0, len(lidar_distances)):
                if lidar_distances[i] == 0:
                    lidar_distances[i] = 100

            min_distance = min(lidar_distances) # get the minimum distance of the filtered lidar data
            #rospy.loginfo('Lidar min dist: %d', min_distance)
            if min_distance < SAFE_STOP_DISTANCE:
                if turtlebot_moving:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    #self._cmd_pub.publish(twist)
                    turtlebot_moving = False
                    rospy.loginfo('Stop!')
                    
                    if twist.linear.x == 0.0:
                        Obstacle_count += 1
                    
                    # Check if there's an obstacle on the left and right
                    left_min_distance = min(lidar_distances[:len(lidar_distances)//2])
                    right_min_distance = min(lidar_distances[len(lidar_distances)//2:])
                    
                    if min_distance < 0.10:
                        twist.linear.x = -0.2
                        twist.angular.z = 0.0
                        rospy.loginfo('Go backward!')
                    if left_min_distance < right_min_distance:
                        twist.linear.x = 0.3 #0.0
                        twist.angular.z = 1.4 #0.5
                        rospy.loginfo('Turn right!')
                    elif right_min_distance < left_min_distance:
                        twist.linear.x = 0.3 #0.0
                        twist.angular.z = -1.4 #0.5
                        rospy.loginfo('Turn left!')
                else:
                    pass
                self._cmd_pub.publish(twist)

            else:
                twist.linear.x = LINEAR_VEL
                twist.angular.z = 0.0
                self._cmd_pub.publish(twist)
                turtlebot_moving = True
                rospy.loginfo(Obstacle_count)
                rospy.loginfo('Distance of the obstacle %f:', min_distance)

def main():
    rospy.init_node('turtlebot3_obstacle')
    try:
        obstacle = Obstacle()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()