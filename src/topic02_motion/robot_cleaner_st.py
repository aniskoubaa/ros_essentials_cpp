#!/usr/bin/env python
 
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time
import math
 
 
 
x_min = 0.0
y_min = 0.0
x_max = 11.0
y_max = 11.0
PI = 3.14159265359
currentTurtlesimPose = Pose()
 
def move(speed, distance,isForward):
    vel_msg = Twist()
    if (isForward):
        vel_msg.linear.x =abs(speed)
    else:
        vel_msg.linear.x =-abs(speed)
    vel_msg.linear.y =0
    vel_msg.linear.z =0
    # set a random angular velocity in the y-axis
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z =0
 
    current_distance = 0
    t0 = rospy.Time.now().to_sec()
    loop_rate = rospy.Rate(100)
 
    while(current_distance < distance):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_distance = (t1-t0)*speed
        loop_rate.sleep()
    
    vel_msg.linear.x = 0
    velocity_publisher.publish(vel_msg)
 
def rotate(angular_speed, relative_angle, clockwise):
    vel_msg = Twist()
    vel_msg.linear.x =0
    vel_msg.linear.y =0
    vel_msg.linear.z =0
    # set a random angular velocity in the y-axis
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
 
    if (clockwise):
        vel_msg.angular.z = -abs(angular_speed)
    else:
        vel_msg.angular.z = abs(angular_speed)
 
    current_angle = 0
    t0 = rospy.Time.now().to_sec()
    loop_rate = rospy.Rate(100)
 
    while(current_angle < relative_angle):
 
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = (t1-t0)*angular_speed
        loop_rate.sleep()
 
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
 
def degrees2radians(angle_in_degrees):
    return angle_in_degrees *PI /180.0
 
 
def poseCallback(pose_message):
    global currentTurtlesimPose
    currentTurtlesimPose = pose_message
 
 
 
def getDistance(x1,y1,x2,y2):
    return ((x1-x2)**2 + (y1-y2)**2)**0.5
 
def setDesiredOrientation(desired_angle_radians):
    relative_angle_radians = desired_angle_radians - currentTurtlesimPose.theta
    if relative_angle_radians < 0:
        clockwise = 1
    else:
        clockwise = 0
    rotate(degrees2radians(20) ,abs(relative_angle_radians), clockwise)
 
def moveGoal(goal_pose, distance_tolerance):
    vel_msg = Twist()
    loop_rate = rospy.Rate(100)
 
    while(getDistance(currentTurtlesimPose.x, currentTurtlesimPose.y, goal_pose.x, goal_pose.y)>distance_tolerance):
 
        Kp = 1
        # Ki = 0.02
        e = getDistance(currentTurtlesimPose.x, currentTurtlesimPose.y, goal_pose.x, goal_pose.y)
        vel_msg.linear.x = (Kp*e)
        # vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        # //angular velocity in the z-axis
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z =4*(math.atan2(goal_pose.y-currentTurtlesimPose.y, goal_pose.x-currentTurtlesimPose.x)-currentTurtlesimPose.theta)
 
 
        velocity_publisher.publish(vel_msg)
        loop_rate.sleep()
 
    vel_msg.linear.x =0
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
 
def gridClean():
 
    desired_pose = Pose()
    desired_pose.x = 1
    desired_pose.y = 1
    desired_pose.theta = 0
 
    moveGoal(desired_pose, 0.01)
 
    setDesiredOrientation(degrees2radians(desired_pose.theta))
 
    move(2.0, 9.0, True)
    rotate(degrees2radians(20), degrees2radians(90), False)
    move(2.0, 9.0, True)
    rotate(degrees2radians(20), degrees2radians(90), False)
    move(2.0, 1.0, True)
    rotate(degrees2radians(20), degrees2radians(90), False)
    move(2.0, 9.0, True)
    rotate(degrees2radians(30), degrees2radians(90), True)
    move(2.0, 1.0, True)
    rotate(degrees2radians(30), degrees2radians(90), True)
    move(2.0, 9.0, True)
    pass
 
 
def spiralClean():
    vel_msg = Twist()
    loop_rate = rospy.Rate(1)
    wk = 4
    rk = 0
 
    while((currentTurtlesimPose.x<10.5) and (currentTurtlesimPose.y<10.5)):
        rk=rk+1
        vel_msg.linear.x =rk
        vel_msg.linear.y =0
        vel_msg.linear.z =0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z =wk
        velocity_publisher.publish(vel_msg)
        loop_rate.sleep()
 
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
 
 
if __name__ == '__main__':
    try:
        rospy.init_node("turtlesim_cleaner", anonymous=True)
        cmd_vel_topic='/turtle1/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)        
        position_topic = "/turtle1/pose"
        pose_subscriber = rospy.Subscriber(position_topic, Pose, poseCallback) 
 
        print "Write 1 for spiral cleaning or 2 for grid cleaning"
        ChosenType = raw_input()
 
        while True:
            if ChosenType == "1":
                print "Grid cleaning was chosen!"
                gridClean()
                # rospy.spin()
                break
            elif ChosenType == "2":
                print "Spiral cleaning was chosen!"
                # rospy.spin()
                spiralClean()
                break
            else:
                print "Type only 1 or 2"
                ChosenType = raw_input()
 
        rospy.spin()
 
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")