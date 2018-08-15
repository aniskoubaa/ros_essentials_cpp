#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import math

def scan_callback(scan_data):
    #Find minimum range
    min_value, min_index = min_range_index(scan_data.ranges)
    print "\nthe minimum range value is: ", min_value
    print "the minimum range index is: ", min_index

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
    return (min(ranges), ranges.index(min(ranges)) )

#find the max range 
def max_range_index(ranges):
    ranges = [x for x in ranges if not math.isnan(x)]
    return (max(ranges), ranges.index(max(ranges)) )

#find the average range
def average_range(ranges):
    ranges = [x for x in ranges if not math.isnan(x)]
    return ( sum(ranges) / float(len(ranges)) )

def average_between_indices(ranges, i, j):
    ranges = [x for x in ranges if not math.isnan(x)]
    slice_of_array = ranges[i: j+1]
    return ( sum(slice_of_array) / float(len(slice_of_array)) )


if __name__ == '__main__':
    
    #init new a node and give it a name
    rospy.init_node('scan_node', anonymous=True)
    #subscribe to the topic /scan. 
    rospy.Subscriber("scan", LaserScan, scan_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()