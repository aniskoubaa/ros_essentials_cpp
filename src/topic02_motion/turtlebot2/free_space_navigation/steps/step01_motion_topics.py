#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion

def poseCallback(pose_message):
    print "pose callback"
    x=0
    print ('x = {}'.format(pose_message.pose.pose.position.y)) #new in python 3
    print ('y = %f' %pose_message.pose.pose.position.y) #used in python 2
    print ('z = {}'.format(pose_message.pose.pose.position.z)) #new in python 3
    print ('yaw = {}'.format(pose_message.pose.pose.position.z)) #new in python 3
    orientation = pose_message.pose.pose.orientation;
    quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
    (roll,pitch,yaw) = euler_from_quaternion(quaternion)
    print ('yaw = {}'.format(yaw)) 

    


if __name__ == '__main__':
    try:
        
        rospy.init_node('node01', anonymous=True)

        #declare velocity publisher
        cmd_vel_topic='/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        
        position_topic = "/odom"
        pose_subscriber = rospy.Subscriber(position_topic, Odometry, poseCallback) 
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")