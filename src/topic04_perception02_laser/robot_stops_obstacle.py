#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import math
from geometry_msgs.msg import Twist
import time


min_distance = 9999

def scan_callback(scan_data):
    global min_distance
    #Find minimum range
    min_value, min_index = min_range_index(scan_data.ranges)
    print "\nthe minimum range value is: ", min_value
    print "the minimum range index is: ", min_index
    min_distance = min_value

    max_value, max_index = max_range_index(scan_data.ranges)
    print "\nthe maximum range value is: ", max_value
    print "the maximum range index is: ", max_index

    average_value = average_range (scan_data.ranges)
    print "\nthe average range value is: ", average_value

    average2 = average_between_indices(scan_data.ranges, 2, 7)
    print "\nthe average between 2 indices is: ", average2

    print "the field of view: ", field_of_view(scan_data)

def field_of_view(scan_data):
    return (scan_data.angle_max-scan_data.angle_min)*180.0/3.14

#find the max range and its index
def min_range_index(ranges):
    ranges = [x for x in ranges if not math.isnan(x)]
    if (len(ranges)!=0):
        return (min(ranges), ranges.index(min(ranges)) )
    else:
        return 0.1
#find the max range 
def max_range_index(ranges):
    ranges = [x for x in ranges if not math.isnan(x)]
    if (len(ranges)!=0):
        return (max(ranges), ranges.index(max(ranges)))
    else:
        return 4.0

#find the average range
def average_range(ranges):
    ranges = [x for x in ranges if not math.isnan(x)]
    if (len(ranges)!=0):
        return ( sum(ranges) / float(len(ranges)) )
    else:
        return 0.0

def average_between_indices(ranges, i, j):
    ranges = [x for x in ranges if not math.isnan(x)]
    if (len(ranges)!=0): 
        slice_of_array = ranges[i: j+1]
        return ( sum(slice_of_array) / float(len(slice_of_array)) )
    else:
        return 0.0


def rotate(avoid_obstacle):
    global min_distance
    velocity_publisher = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=10)

    cmd_vel = Twist()
    cmd_vel.linear.x = 0.0
    cmd_vel.angular.z = 0.6
    loop_rate = rospy.Rate(10)
    if (avoid_obstacle):
        while (min_distance<1.5):
            velocity_publisher.publish(cmd_vel)
            loop_rate.sleep()

        #force the robot to step before avoiding obstacles
        cmd_vel.angular.z = 0.0
        velocity_publisher.publish(cmd_vel)
    else:
        velocity_publisher.publish(cmd_vel)
        loop_rate.sleep() 

def move(avoid_obstacle):
    #create a publisher to make the robot move
    global min_distance
    velocity_publisher = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=10)

    cmd_vel = Twist()
    
    loop_rate = rospy.Rate(10)
    while(True):
        if (avoid_obstacle):
            #behavior 1 
            cmd_vel.linear.x = 0.3
            cmd_vel.angular.z = 0.0
            while (min_distance>0.7):
                velocity_publisher.publish(cmd_vel)
                loop_rate.sleep()

            #behavior 2: force the robot rotate until it finds open space
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 1.5
            while (min_distance<1.5):
                velocity_publisher.publish(cmd_vel)
                loop_rate.sleep()

        else:
            velocity_publisher.publish(cmd_vel)
            loop_rate.sleep() 

if __name__ == '__main__':
    
    #init new a node and give it a name
    rospy.init_node('scan_node', anonymous=True)
    #subscribe to the topic /scan. 
    rospy.Subscriber("scan", LaserScan, scan_callback)

    time.sleep(2)
    move(True)
    rotate(True)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()