#!/usr/bin/env python

import rospy
import tf
import numpy
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from math import *
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import tf2_ros
import copy

LINEAR_VELOCITY_MINIMUM_THRESHOLD  = 0.2
ANGULAR_VELOCITY_MINIMUM_THRESHOLD = 0.4
class free_space_navigation():

    

    def poseCallback(self,pose_message):

        self.turtlebot_odom_pose.pose.pose.position.x=pose_message.pose.pose.position.x
        self.turtlebot_odom_pose.pose.pose.position.y=pose_message.pose.pose.position.y
        self.turtlebot_odom_pose.pose.pose.position.z=pose_message.pose.pose.position.z

        self.turtlebot_odom_pose.pose.pose.orientation.w=pose_message.pose.pose.orientation.w
        self.turtlebot_odom_pose.pose.pose.orientation.x=pose_message.pose.pose.orientation.x
        self.turtlebot_odom_pose.pose.pose.orientation.y=pose_message.pose.pose.orientation.y
        self.turtlebot_odom_pose.pose.pose.orientation.z=pose_message.pose.pose.orientation.z

 # a function that makes the robot move straight
 # @param speed: represents the speed of the robot the robot
 # @param distance: represents the distance to move by the robot
 # @param isForward: if True, the robot moves forward,otherwise, it moves backward
 #
 # Method 1: using tf and Calculate the distance between the two transformations

    def move_v1(self, speed, distance, isForward):
        #declare a Twist message to send velocity commands
        VelocityMessage = Twist()
        # declare tf transform listener: this transform listener will be used to listen and capture the transformation between
        # the odom frame (that represent the reference frame) and the base_footprint frame the represent moving frame
        listener = tf.TransformListener()

        #set the linear velocity to a positive value if isFoward is True
        if (isForward):
            VelocityMessage.linear.x =abs(speed)
        else: #else set the velocity to negative value to move backward
            VelocityMessage.linear.x =-abs(speed)

        # all velocities of other axes must be zero.
        VelocityMessage.linear.y =0
        VelocityMessage.linear.z =0
        #The angular velocity of all axes must be zero because we want  a straight motion
        VelocityMessage.angular.x = 0
        VelocityMessage.angular.y = 0
        VelocityMessage.angular.z =0

        distance_moved = 0.0
        loop_rate = rospy.Rate(10) # we publish the velocity at 10 Hz (10 times a second)    


     #  First, we capture the initial transformation before starting the motion.
     # we call this transformation "init_transform"
     # It is important to "waitForTransform" otherwise, it might not be captured.

        try:
            #wait for the transform to be found

            listener.waitForTransform("/base_footprint", "/odom", rospy.Time(0),rospy.Duration(10.0))
            #Once the transform is found,get the initial_transform transformation.
            (trans,rot) = listener.lookupTransform('/base_footprint', '/odom', rospy.Time(0))
            #listener.lookupTransform("/base_footprint", "/odom", rospy.Time(0),init_transform)
            start = 0.5 * sqrt(trans[0] ** 2 + trans[1] ** 2)

        except Exception:
            rospy.Duration(1.0)

        distance_moved = 0
        
        while True :
            rospy.loginfo("Turtlebot moves forwards") 
        #/***************************************
        # * STEP1. PUBLISH THE VELOCITY MESSAGE
        # ***************************************/
            self.velocityPublisher.publish(VelocityMessage)
            loop_rate.sleep()
        #/**************************************************
        # * STEP2. ESTIMATE THE DISTANCE MOVED BY THE ROBOT
        # *************************************************/
            try:

                #wait for the transform to be found
                listener.waitForTransform("/base_footprint", "/odom", rospy.Time(0), rospy.Duration(10.0) )
                #Once the transform is found,get the initial_transform transformation.
                #listener.lookupTransform("/base_footprint", "/odom",rospy.Time(0))
                (trans,rot) = listener.lookupTransform('/base_footprint', '/odom', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.Duration(1.0)
        # Calculate the distance moved by the robot
        # There are two methods that give the same result
        #
        # Method 1: Calculate the distance between the two transformations
        # Hint:
        #    --> transform.getOrigin().x(): represents the x coordinate of the transformation
        #    --> transform.getOrigin().y(): represents the y coordinate of the transformation
        #
        # calculate the distance moved
            end = 0.5 * sqrt(trans[0] ** 2 + trans[1] ** 2)
            distance_moved = distance_moved+abs(abs(float(end)) - abs(float(start)))
            if not (distance_moved<distance):
                break
            
            #finally, stop the robot when the distance is moved
        VelocityMessage.linear.x =0 
        self.velocityPublisher.publish(VelocityMessage)

    
    def move_v2(self, speed, distance, isForward):

        #declare a Twist message to send velocity commands
        VelocityMessage = Twist()
        # declare tf transform listener: this transform listener will be used to listen and capture the transformation between
        # the odom frame (that represent the reference frame) and the base_footprint frame the represent moving frame
        # set the linear velocity to a positive value if isFoward is True
        listener = tf.TransformListener()
        #declare tf transform
        initial_turtlebot_odom_pose = Odometry()
        #init_transform: is the transformation before starting the motion
        init_transform = geometry_msgs.msg.TransformStamped()
        #current_transformation: is the transformation while the robot is moving
        current_transform = geometry_msgs.msg.TransformStamped()

        if (isForward):
            VelocityMessage.linear.x =abs(speed)
        else: #else set the velocity to negative value to move backward
            VelocityMessage.linear.x =-abs(speed)
        #all velocities of other axes must be zero.
        VelocityMessage.linear.y =0.0
        VelocityMessage.linear.z =0.0
        VelocityMessage.angular.x =0.0
        VelocityMessage.angular.y =0.0
        VelocityMessage.angular.z =0.0

        distance_moved = 0.0

        loop_rate = rospy.Rate(10) # we publish the velocity at 10 Hz (10 times a second)

     # First, we capture the initial transformation before starting the motion.
     # we call this transformation "init_transform"
     # It is important to "waitForTransform" otherwise, it might not be captured.
        try:
            #wait for the transform to be found

            listener.waitForTransform("/base_footprint", "/odom", rospy.Time(0),rospy.Duration(10.0))
            #Once the transform is found,get the initial_transform transformation.
            (trans,rot) = listener.lookupTransform('/base_footprint', '/odom', rospy.Time(0))
            #listener.lookupTransform("/base_footprint", "/odom", rospy.Time(0),init_transform)
            trans1_mat = tf.transformations.translation_matrix(trans)
            rot1_mat   = tf.transformations.quaternion_matrix(rot)
            mat1 = numpy.dot(trans1_mat, rot1_mat)
            init_transform.transform.translation = trans
            init_transform.transform.rotation =rot
            #print(mat1)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.Duration(1.0)
     
        while True :
            rospy.loginfo("Turtlebot moves forwards") 
        #/***************************************
        # * STEP1. PUBLISH THE VELOCITY MESSAGE
        # ***************************************/
            self.velocityPublisher.publish(VelocityMessage)
            loop_rate.sleep()
        #/**************************************************
        # * STEP2. ESTIMATE THE DISTANCE MOVED BY THE ROBOT
        # *************************************************/
            try:

                #wait for the transform to be found
                listener.waitForTransform("/base_footprint", "/odom", rospy.Time(0), rospy.Duration(10.0) )
                #Once the transform is found,get the initial_transform transformation.
                #listener.lookupTransform("/base_footprint", "/odom",rospy.Time(0))
                (trans,rot) = listener.lookupTransform('/base_footprint', '/odom', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.Duration(1.0)
            
            trans1_mat = tf.transformations.translation_matrix(trans)
            rot1_mat   = tf.transformations.quaternion_matrix(rot)
            mat2 = numpy.dot(trans1_mat, rot1_mat)
            #print(mat2)
            mat3 = numpy.dot(mat1, mat2)
            #print(mat3)

            trans3 = tf.transformations.translation_from_matrix(mat3)
            
            #print(trans3)
            rot3 = tf.transformations.quaternion_from_matrix(mat3)
            #print(rot3)
            
            current_transform.transform.translation = trans
            current_transform.transform.rotation =rot
            distance_moved = distance_moved + (0.5 * sqrt(trans3[0] ** 2 + trans3[1] ** 2))
            #print(distance_moved)
            #print (numpy.linalg.norm(trans3))
            
            if not (distance_moved<distance):
                break
            
    #finally, stop the robot when the distance is moved
        VelocityMessage.linear.x =0
        self.velocityPublisher.publish(VelocityMessage)
    


    # a function that makes the robot move straight
    # @param speed: represents the speed of the robot the robot
    # @param distance: represents the distance to move by the robot
    # @param isForward: if True, the robot moves forward,otherwise, it moves backward
    #
    # Method 3: we use coordinates of the robot to estimate the distance

    def move_v3(self, speed, distance, isForward):
        #declare a Twist message to send velocity commands
            VelocityMessage = Twist()
        # declare tf transform listener: this transform listener will be used to listen and capture the transformation between
        # the odom frame (that represent the reference frame) and the base_footprint frame the represent moving frame
            listener = tf.TransformListener()
        #declare tf transform
            initial_turtlebot_odom_pose = Odometry()
        #init_transform: is the transformation before starting the motion
            init_transform = geometry_msgs.msg.TransformStamped()
        #current_transformation: is the transformation while the robot is moving
            current_transform = geometry_msgs.msg.TransformStamped()

        #set the linear velocity to a positive value if isFoward is True
            if (isForward):
                    VelocityMessage.linear.x =abs(speed)
            else: #else set the velocity to negative value to move backward
                    VelocityMessage.linear.x =-abs(speed)

        # all velocities of other axes must be zero.
            VelocityMessage.linear.y =0
            VelocityMessage.linear.z =0
        #The angular velocity of all axes must be zero because we want  a straight motion
            VelocityMessage.angular.x = 0
            VelocityMessage.angular.y = 0
            VelocityMessage.angular.z = 0

            distance_moved = 0.0
            loop_rate = rospy.Rate(20) # we publish the velocity at 10 Hz (10 times a second)    
            #we update the initial_turtlebot_odom_pose using the turtlebot_odom_pose global variable updated in the callback function poseCallback
            #we will use deepcopy() to avoid pointers confusion
            initial_turtlebot_odom_pose = copy.deepcopy(self.turtlebot_odom_pose)

            while True :
                    rospy.loginfo("Turtlebot moves forwards")
                    self.velocityPublisher.publish(VelocityMessage)
         
                    loop_rate.sleep()
                    
                    #rospy.Duration(1.0)
                    
                    distance_moved = distance_moved+abs(0.5 * sqrt(((self.turtlebot_odom_pose.pose.pose.position.x-initial_turtlebot_odom_pose.pose.pose.position.x) ** 2) +
                        ((self.turtlebot_odom_pose.pose.pose.position.x-initial_turtlebot_odom_pose.pose.pose.position.x) ** 2)))
                                        
                    if not (distance_moved<distance):
                        break
            
            #finally, stop the robot when the distance is moved
            VelocityMessage.linear.x =0
            self.velocityPublisher.publish(VelocityMessage)

    def degree2radian(self, degreeAngle):
        return (degreeAngle/57.2957795)
    
    def rotate(self,angular_velocity,radians,clockwise):
        rotateMessage = Twist()
       
        #declare tf transform 
        listener = tf.TransformListener()
        #init_transform: is the transformation before starting the motion
        init_transform = geometry_msgs.msg.TransformStamped()
        #current_transformation: is the transformation while the robot is moving
        current_transform = geometry_msgs.msg.TransformStamped()
        
        angle_turned = 0.0

        angular_velocity = (-angular_velocity, ANGULAR_VELOCITY_MINIMUM_THRESHOLD)[angular_velocity > ANGULAR_VELOCITY_MINIMUM_THRESHOLD]

        while(radians < 0):
            radians += 2* pi

        while(radians > 2* pi):
            radians -= 2* pi
        
        listener.waitForTransform("/base_footprint", "/odom", rospy.Time(0), rospy.Duration(10.0) )
        (trans,rot) = listener.lookupTransform('/base_footprint', '/odom', rospy.Time(0))
        #listener.lookupTransform("/base_footprint", "/odom", rospy.Time(0),init_transform)
        
        init_transform.transform.translation = trans
        init_transform.transform.rotation =rot

        #since the rotation is only in the Z-axes 
        #start_angle = tf.transformations.#0.5 * sqrt(rot[2] ** 2)
        euler = tf.transformations.euler_from_quaternion(rot)
        roll = euler[0]
        pitch = euler[1]
        start_angle = euler[2]

        rotateMessage.linear.x = rotateMessage.linear.y = 0.0
        rotateMessage.angular.z = angular_velocity

        if (clockwise):
            rotateMessage.angular.z = -rotateMessage.angular.z
        
        
        loop_rate = rospy.Rate(20)
        
        while True:
            rospy.loginfo("Turtlebot is Rotating")

            self.velocityPublisher.publish(rotateMessage)
         
            loop_rate.sleep()
                    
            #rospy.Duration(1.0)

            try:

                #wait for the transform to be found
                listener.waitForTransform("/base_footprint", "/odom", rospy.Time(0), rospy.Duration(10.0) )
                #Once the transform is found,get the initial_transform transformation.
                #listener.lookupTransform("/base_footprint", "/odom",rospy.Time(0))
                (trans,rot) = listener.lookupTransform('/base_footprint', '/odom', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.Duration(1.0)

            current_transform.transform.translation = trans
            current_transform.transform.rotation =rot

            #since the rotation is only in the Z-axes 
            #end_angle = 0.5 * sqrt( rot[2] ** 2)
            euler = tf.transformations.euler_from_quaternion(rot)
            roll = euler[0]
            pitch = euler[1]
            end_angle = euler[2]
            
            angle_turned = abs(end_angle - start_angle)
            print "angle_turned: %s" %angle_turned
            if (angle_turned > radians):
                break

        self.velocityPublisher.publish(rotateMessage)
        loop_rate.sleep()

    def calculateYaw(self, x1, y1, x2,y2):
        bearing = atan2((y2 - y1),(x2 - x1))
        #if(bearing < 0) bearing += 2 * PI
        bearing *= 180.0 / pi
        return bearing


    def moveSquare(self,sideLength):
        for i in range(0, 4):
            self.move_v1(0.3, sideLength, True)
            #self.shutdown()
            self.rotate(0.2, self.degree2radian(90.0),True)
   
    def __init__(self):
        # initiliaze
        rospy.init_node('free_space_navigation', anonymous=True)

        # What to do you ctrl + c    
        rospy.on_shutdown(self.shutdown)
        self.turtlebot_odom_pose = Odometry()
        pose_message = Odometry()
        self.velocityPublisher = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber("/odom", Odometry, self.poseCallback)     
        # 2 HZ
        r = rospy.Rate(20)
        
        r.sleep()

        while not rospy.is_shutdown():
            sideLength=1
            self.moveSquare(sideLength)
            #self.rotate(0.3, self.degree2radian(90.0), True);
            #self.rotate(0.3, self.degree2radian(90.0), False);       

        
    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop Drawing Squares")
        self.velocityPublisher.publish(Twist())
        rospy.sleep(1)
    
 

    

if __name__ == '__main__':
    try:
        free_space_navigation()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
