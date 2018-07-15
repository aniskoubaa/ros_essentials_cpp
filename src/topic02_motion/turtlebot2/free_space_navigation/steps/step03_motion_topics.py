#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion
import math

x=0
y=0
z=0
yaw=0

def poseCallback(pose_message):
    #print "pose callback"
    global x
    global y, z, yaw
    x=pose_message.pose.pose.position.x
    y=pose_message.pose.pose.position.y
    z=pose_message.pose.pose.position.z
    orientation = pose_message.pose.pose.orientation;
    quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
    (roll,pitch,yaw) = euler_from_quaternion(quaternion)


# a function that makes the robot move straight
    # @param speed: represents the speed of the robot the robot
    # @param distance: represents the distance to move by the robot
    # @param isForward: if True, the robot moves forward,otherwise, it moves backward
    #
    # we use coordinates of the robot to estimate the distance

def move(speed, distance, isForward):
        #declare a Twist message to send velocity commands
            velocity_message = Twist()


            #get current location 
            x0=x;
            y0=y;
            #z0=z;
            #yaw0=yaw;
            velocity_message.linear.x =speed
            distance_moved = 0.0
            loop_rate = rospy.Rate(10) # we publish the velocity at 10 Hz (10 times a second)    
            cmd_vel_topic='/cmd_vel'
            velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

            while True :
                    rospy.loginfo("Turtlebot moves forwards")
                    velocity_publisher.publish(velocity_message)
         
                    loop_rate.sleep()
                    
                    #rospy.Duration(1.0)
                    
                    distance_moved = distance_moved+abs(0.5 * math.sqrt(((x-x0) ** 2) + ((y-y0) ** 2)))
                    print  distance_moved               
                    if not (distance_moved<distance):
                        rospy.loginfo("reached")
                        break
            
            #finally, stop the robot when the distance is moved
            velocity_message.linear.x =0
            velocity_publisher.publish(velocity_message)
    
def rotate(angular_velocity,radians,clockwise):
    velocity_message = Twist()
    #get current location 
    #x0=x;
    #y0=y;
    #z0=z;
    yaw0=yaw;

    rotated_angle = 0;
    print  rotated_angle 
    loop_rate = rospy.Rate(10) # we publish the velocity at 10 Hz (10 times a second)    
    cmd_vel_topic='/cmd_vel_mux/input/teleop'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

    velocity_message.linear.x =0
    if (clockwise):
        velocity_message.angular.z = -angular_velocity
    else:
        velocity_message.angular.z = angular_velocity

    while True :
            rospy.loginfo("Turtlebot moves rotates")
            velocity_publisher.publish(velocity_message)
    
            loop_rate.sleep()
            
            #rospy.Duration(1.0)
            
            rotated_angle = abs(yaw-yaw0)
            print  rotated_angle               
            if not (rotated_angle<radians):
                rospy.loginfo("reached")
                break
    
    #finally, stop the robot when the distance is moved
    velocity_message.angular.z =0
    velocity_publisher.publish(velocity_message)

    

if __name__ == '__main__':
    try:
        
        rospy.init_node('node01', anonymous=True)

        #declare velocity publisher
        cmd_vel_topic='/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        
        position_topic = "/odom"
        pose_subscriber = rospy.Subscriber(position_topic, Odometry, poseCallback) 
        
        #move(1.0,10.0,True)
        rotate (0.3, 1.57, 1)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")