/*
 *  Gaitech Educational Portal
 *
 * Copyright (c) 2016
 * All rights reserved.
 *
 * License Type: GNU GPL
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *    Program: Free-Space Navigation
 *
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include <tf/transform_listener.h>
#include <fstream>

using namespace std;
#define LINEAR_VELOCITY_MINIMUM_THRESHOLD 0.2
#define ANGULAR_VELOCITY_MINIMUM_THRESHOLD 0.4

//declare publishers
ros::Publisher velocityPublisher;
//declare subscribers
ros::Subscriber scanSubscriber;
ros::Subscriber pose_subscriber;
//global variable to update the position of the robot
nav_msgs::Odometry turtlebot_odom_pose;




//callback function for the /odom topic to update the pose
void poseCallback(const nav_msgs::Odometry::ConstPtr & pose_message);
//the function that makes the robot moves forward and backward
void move_v1(double speed, double distance, bool isForward);
void move_v2(double speed, double distance, bool isForward);
void move_v3(double speed, double distance, bool isForward);
//the function that makes the robot rotates left and right
double rotate(double ang_vel, double angle_radian, bool isClockwise);
double degree2radian(double degreeAngle);
double radian2degree(double radianAngle);
double calculateYaw( double x1, double y1, double x2,double y2);

void moveSquare(double sideLength);

int main(int argc, char **argv){

	//initialize the ROS node
	ros::init(argc, argv, "free_space_navigation_node");
	ros::NodeHandle n;

	//subscribe to the odometry topic to get the position of the robot
	pose_subscriber = n.subscribe("/odom", 10, poseCallback);
	//register the velocity publisher
	velocityPublisher =n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1000);

	ros::spinOnce();
	ros::Rate loop(1);
	loop.sleep();loop.sleep();loop.sleep();//loop.sleep();loop.sleep();
	ros::spinOnce();

	while (ros::ok()){
		ros::spinOnce();loop.sleep();
		printf("robot initial pose: (%.2f, %.2f, %.2f)\n",
										turtlebot_odom_pose.pose.pose.position.x,
										turtlebot_odom_pose.pose.pose.position.y,
										radian2degree(tf::getYaw(turtlebot_odom_pose.pose.pose.orientation)));
		moveSquare(1.0);
		//exercise: try to remove the ros::SpinOnce() and observe and comment the result
		ros::spinOnce();loop.sleep();ros::spinOnce();
		printf("robot final pose: (%.2f, %.2f, %.2f)\n",
											turtlebot_odom_pose.pose.pose.position.x,
											turtlebot_odom_pose.pose.pose.position.y,
											radian2degree(tf::getYaw(turtlebot_odom_pose.pose.pose.orientation)));
		return 0;
	}



	return 0;
}




void poseCallback(const nav_msgs::Odometry::ConstPtr & pose_message){
	turtlebot_odom_pose.pose.pose.position.x=pose_message->pose.pose.position.x;
	turtlebot_odom_pose.pose.pose.position.y=pose_message->pose.pose.position.y;
	turtlebot_odom_pose.pose.pose.position.z=pose_message->pose.pose.position.z;

	turtlebot_odom_pose.pose.pose.orientation.w=pose_message->pose.pose.orientation.w;
	turtlebot_odom_pose.pose.pose.orientation.x=pose_message->pose.pose.orientation.x;
	turtlebot_odom_pose.pose.pose.orientation.y=pose_message->pose.pose.orientation.y;
	turtlebot_odom_pose.pose.pose.orientation.z=pose_message->pose.pose.orientation.z;
}

void moveSquare(double sideLength){
	for (int i=0;i<4;i++){
		move_v1(0.3, sideLength, true);
		rotate (0.3, degree2radian(90), true);
	}
}


/**
 * a function that makes the robot move straight
 * @param speed: represents the speed of the robot the robot
 * @param distance: represents the distance to move by the robot
 * @param isForward: if true, the robot moves forward,otherwise, it moves backward
 *
 * Method 1: using tf and Calculate the distance between the two transformations
 */
void move_v1(double speed, double distance, bool isForward){
	//declare a Twist message to send velocity commands
	geometry_msgs::Twist VelocityMessage;
	//declare tf transform listener: this transform listener will be used to listen and capture the transformation between
	// the /odom frame (that represent the reference frame) and the base_footprint frame the represent moving frame
	tf::TransformListener listener;
	//declare tf transform
	//init_transform: is the transformation before starting the motion
	tf::StampedTransform init_transform;
	//current_transformation: is the transformation while the robot is moving
	tf::StampedTransform current_transform;


	//set the linear velocity to a positive value if isFoward is true
	if (isForward)
		VelocityMessage.linear.x =abs(speed);
	else //else set the velocity to negative value to move backward
		VelocityMessage.linear.x =-abs(speed);
	//all velocities of other axes must be zero.
	VelocityMessage.linear.y =0;
	VelocityMessage.linear.z =0;
	//The angular velocity of all axes must be zero because we want  a straight motion
	VelocityMessage.angular.x = 0;
	VelocityMessage.angular.y = 0;
	VelocityMessage.angular.z =0;

	double distance_moved = 0.0;
	ros::Rate loop_rate(10); // we publish the velocity at 10 Hz (10 times a second)

	/*
	 * First, we capture the initial transformation before starting the motion.
	 * we call this transformation "init_transform"
	 * It is important to "waitForTransform" otherwise, it might not be captured.
	 */
	try{
		//wait for the transform to be found
		listener.waitForTransform("/base_footprint", "/odom", ros::Time(0), ros::Duration(10.0) );
		//Once the transform is found,get the initial_transform transformation.
		listener.lookupTransform("/base_footprint", "/odom",ros::Time(0), init_transform);
	}
	catch (tf::TransformException & ex){
		ROS_ERROR(" Problem %s",ex.what());
		ros::Duration(1.0).sleep();
	}



	do{
		/***************************************
		 * STEP1. PUBLISH THE VELOCITY MESSAGE
		 ***************************************/
		velocityPublisher.publish(VelocityMessage);
		ros::spinOnce();
		loop_rate.sleep();
		/**************************************************
		 * STEP2. ESTIMATE THE DISTANCE MOVED BY THE ROBOT
		 *************************************************/
		try{
			//wait for the transform to be found
			listener.waitForTransform("/base_footprint", "/odom", ros::Time(0), ros::Duration(10.0) );
			//Once the transform is found,get the initial_transform transformation.
			listener.lookupTransform("/base_footprint", "/odom",ros::Time(0), current_transform);
		}
		catch (tf::TransformException & ex){
			ROS_ERROR(" Problem %s",ex.what());
			ros::Duration(1.0).sleep();
		}
		/*
		 * Calculate the distance moved by the robot
		 * There are two methods that give the same result
		 */

		/*
		 * Method 1: Calculate the distance between the two transformations
		 * Hint:
		 * 	  --> transform.getOrigin().x(): represents the x coordinate of the transformation
		 * 	  --> transform.getOrigin().y(): represents the y coordinate of the transformation
		 */
		//calculate the distance moved
		//cout<<"Initial Transform: "<<init_transform <<" , "<<"Current Transform: "<<current_transform<<endl;

		distance_moved = sqrt(pow((current_transform.getOrigin().x()-init_transform.getOrigin().x()), 2) +
				pow((current_transform.getOrigin().y()-init_transform.getOrigin().y()), 2));


	}while((distance_moved<distance)&&(ros::ok()));
	//finally, stop the robot when the distance is moved
	VelocityMessage.linear.x =0;
	velocityPublisher.publish(VelocityMessage);
}

/**
 * a function that makes the robot move straight
 * @param speed: represents the speed of the robot the robot
 * @param distance: represents the distance to move by the robot
 * @param isForward: if true, the robot moves forward,otherwise, it moves backward
 *
 * Method 2: using tf and we calculate the relative transform, then we determine its length
 */
void move_v2(double speed, double distance, bool isForward){
	//delcare a Twist message to send velocity commands
	geometry_msgs::Twist VelocityMessage;
	//declare tf transform listener: this transform listener will be used to listen and capture the transformation between
	// the odom frame (that represent the reference frame) and the base_footprint frame the represent moving frame
	tf::TransformListener listener;
	//declare tf transform
	//init_transform: is the transformation before starting the motion
	tf::StampedTransform init_transform;
	//current_transformation: is the transformation while the robot is moving
	tf::StampedTransform current_transform;
	//initial coordinates (for method 3)
	nav_msgs::Odometry initial_turtlebot_odom_pose;

	//set the linear velocity to a positive value if isFoward is true
	if (isForward)
		VelocityMessage.linear.x =abs(speed);
	else //else set the velocity to negative value to move backward
		VelocityMessage.linear.x =-abs(speed);
	//all velocities of other axes must be zero.
	VelocityMessage.linear.y = VelocityMessage.linear.z =VelocityMessage.angular.x =VelocityMessage.angular.y =VelocityMessage.angular.z =0;

	double distance_moved = 0.0;
	ros::Rate loop_rate(10); // we publish the velocity at 10 Hz (10 times a second)

	/*
	 * First, we capture the initial transformation before starting the motion.
	 * we call this transformation "init_transform"
	 * It is important to "waitForTransform" otherwise, it might not be captured.
	 */
	try{
		//wait for the transform to be found
		listener.waitForTransform("/base_footprint", "/odom", ros::Time(0), ros::Duration(10.0) );
		//Once the transform is found,get the initial_transform transformation.
		listener.lookupTransform("/base_footprint", "/odom",ros::Time(0), init_transform);
	}
	catch (tf::TransformException & ex){
		ROS_ERROR(" Problem %s",ex.what());
		ros::Duration(1.0).sleep();
	}



	do{
		/***************************************
		 * STEP1. PUBLISH THE VELOCITY MESSAGE
		 ***************************************/
		velocityPublisher.publish(VelocityMessage);
		ros::spinOnce();
		loop_rate.sleep();
		/**************************************************
		 * STEP2. ESTIMATE THE DISTANCE MOVED BY THE ROBOT
		 *************************************************/
		try{
			//wait for the transform to be found
			listener.waitForTransform("/base_footprint", "/odom", ros::Time(0), ros::Duration(10.0) );
			//Once the transform is found,get the initial_transform transformation.
			listener.lookupTransform("/base_footprint", "/odom",ros::Time(0), current_transform);
		}
		catch (tf::TransformException & ex){
			ROS_ERROR(" Problem %s",ex.what());
			ros::Duration(1.0).sleep();
		}

		/*
		 * Method 2: using transform composition. We calculate the relative transform, then we determine its length
		 * Hint:
		 * 	  --> transform.getOrigin().length(): return the displacement of the origin of the transformation
		 */
		tf::Transform relative_transform = init_transform.inverse() * current_transform;
		distance_moved= relative_transform.getOrigin().length();

		//cout<<"Method 2: distance moved: "<<distance_moved <<", "<<distance<<endl;

	}while((distance_moved<distance)&&(ros::ok()));
	//finally, stop the robot when the distance is moved
	VelocityMessage.linear.x =0;
	velocityPublisher.publish(VelocityMessage);
}


/**
 * a function that makes the robot move straight
 * @param speed: represents the speed of the robot the robot
 * @param distance: represents the distance to move by the robot
 * @param isForward: if true, the robot moves forward,otherwise, it moves backward
 *
 * Method 3: we use coordinates of the robot to estimate the distance
 */
void move_v3(double speed, double distance, bool isForward){
	//delcare a Twist message to send velocity commands
	geometry_msgs::Twist VelocityMessage;
	//initial pose of the turtlebot before start moving
	nav_msgs::Odometry initial_turtlebot_odom_pose;

	//set the linear velocity to a positive value if isFoward is true
	if (isForward)
		VelocityMessage.linear.x =abs(speed);
	else //else set the velocity to negative value to move backward
		VelocityMessage.linear.x =-abs(speed);
	//all velocities of other axes must be zero.
	VelocityMessage.linear.y = VelocityMessage.linear.z =VelocityMessage.angular.x =VelocityMessage.angular.y =VelocityMessage.angular.z =0;

	double distance_moved = 0.0;
	ros::Rate loop_rate(10); // we publish the velocity at 10 Hz (10 times a second)

	//we update the initial_turtlebot_odom_pose using the turtlebot_odom_pose global variable updated in the callback function poseCallback
	initial_turtlebot_odom_pose = turtlebot_odom_pose;

	do{
		velocityPublisher.publish(VelocityMessage);
		ros::spinOnce();
		loop_rate.sleep();
		distance_moved = sqrt(pow((turtlebot_odom_pose.pose.pose.position.x-initial_turtlebot_odom_pose.pose.pose.position.x), 2) +
				pow((turtlebot_odom_pose.pose.pose.position.y-initial_turtlebot_odom_pose.pose.pose.position.y), 2));

	}while((distance_moved<distance)&&(ros::ok()));
	//finally, stop the robot when the distance is moved
	VelocityMessage.linear.x =0;
	velocityPublisher.publish(VelocityMessage);
}



double rotate(double angular_velocity, double radians,  bool clockwise)
{

	//delcare a Twist message to send velocity commands
	geometry_msgs::Twist VelocityMessage;
	//declare tf transform listener: this transform listener will be used to listen and capture the transformation between
	// the odom frame (that represent the reference frame) and the base_footprint frame the represent moving frame
	tf::TransformListener TFListener;
	//declare tf transform
	//init_transform: is the transformation before starting the motion
	tf::StampedTransform init_transform;
	//current_transformation: is the transformation while the robot is moving
	tf::StampedTransform current_transform;
	//initial coordinates (for method 3)
	nav_msgs::Odometry initial_turtlebot_odom_pose;

	double angle_turned =0.0;

	//validate angular velocity; ANGULAR_VELOCITY_MINIMUM_THRESHOLD is the minimum allowed
	angular_velocity=((angular_velocity>ANGULAR_VELOCITY_MINIMUM_THRESHOLD)?angular_velocity:ANGULAR_VELOCITY_MINIMUM_THRESHOLD);

	while(radians < 0) radians += 2*M_PI;
	while(radians > 2*M_PI) radians -= 2*M_PI;

	//wait for the listener to get the first message
	TFListener.waitForTransform("base_footprint", "odom", ros::Time(0), ros::Duration(1.0));


	//record the starting transform from the odometry to the base frame
	TFListener.lookupTransform("base_footprint", "odom", ros::Time(0), init_transform);


	//the command will be to turn at 0.75 rad/s
	VelocityMessage.linear.x = VelocityMessage.linear.y = 0.0;
	VelocityMessage.angular.z = angular_velocity;
	if (clockwise) VelocityMessage.angular.z = -VelocityMessage.angular.z;

	//the axis we want to be rotating by
	tf::Vector3 desired_turn_axis(0,0,1);
	if (!clockwise) desired_turn_axis = -desired_turn_axis;

	ros::Rate rate(10.0);
	bool done = false;
	while (!done )
	{
		//send the drive command
		velocityPublisher.publish(VelocityMessage);
		rate.sleep();
		//get the current transform
		try
		{
			TFListener.waitForTransform("base_footprint", "odom", ros::Time(0), ros::Duration(1.0));
			TFListener.lookupTransform("base_footprint", "odom", ros::Time(0), current_transform);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s",ex.what());
			break;
		}
		tf::Transform relative_transform = init_transform.inverse() * current_transform;
		tf::Vector3 actual_turn_axis = relative_transform.getRotation().getAxis();
		angle_turned = relative_transform.getRotation().getAngle();

		if (fabs(angle_turned) < 1.0e-2) continue;
		if (actual_turn_axis.dot(desired_turn_axis ) < 0 )
			angle_turned = 2 * M_PI - angle_turned;

		if (!clockwise)
			VelocityMessage.angular.z = (angular_velocity-ANGULAR_VELOCITY_MINIMUM_THRESHOLD) * (fabs(radian2degree(radians-angle_turned)/radian2degree(radians)))+ANGULAR_VELOCITY_MINIMUM_THRESHOLD;
		else
			if (clockwise)
				VelocityMessage.angular.z = (-angular_velocity+ANGULAR_VELOCITY_MINIMUM_THRESHOLD) * (fabs(radian2degree(radians-angle_turned)/radian2degree(radians)))-ANGULAR_VELOCITY_MINIMUM_THRESHOLD;

		if (angle_turned > radians) {
			done = true;
			VelocityMessage.linear.x = VelocityMessage.linear.y = VelocityMessage.angular.z = 0;
			velocityPublisher.publish(VelocityMessage);
		}


	}
	if (done) return angle_turned;
	return angle_turned;
}


double calculateYaw( double x1, double y1, double x2,double y2)
{

	double bearing = atan2((y2 - y1),(x2 - x1));
	//if(bearing < 0) bearing += 2 * PI;
	bearing *= 180.0 / M_PI;
	return bearing;
}

/* makes conversion from radian to degree */
double radian2degree(double radianAngle){
	return (radianAngle*57.2957795);
}


/* makes conversion from degree to radian */
double degree2radian(double degreeAngle){
	return (degreeAngle/57.2957795);
}





