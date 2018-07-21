/*
 * Author: Anis Koubaa
 * Year: 2018
 * Prince Sultan University
 * Robotics and Internet of Things Lab
 */


#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <sstream>

using namespace std;

ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
turtlesim::Pose turtlesim_pose;


const double x_min = 0.0;
const double y_min = 0.0;
const double x_max = 11.0;
const double y_max = 11.0;

const double PI = 3.14159265359;

void move(double speed, double distance, bool isForward);
void rotate (double angular_speed, double angle, bool clockwise);
double degrees2radians(double angle_in_degrees);
void setDesiredOrientation (double desired_angle_radians);
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);
void moveGoal(turtlesim::Pose  goal_pose, double distance_tolerance);
void gridClean();
void spiralClean();

int main(int argc, char **argv)
{
	// Initiate new ROS node named "talker"
	ros::init(argc, argv, "turtlesim_cleaner");
	ros::NodeHandle n;
	double speed, angular_speed;
	double distance, angle;
	bool isForward, clockwise;


	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);

	//ros::Rate loop_rate(10);



	/** test your code here **/
	ROS_INFO("\n\n\n******START TESTING************\n");
	/*cout<<"enter speed: ";
	cin>>speed;
	cout<<"enter distance: ";
	cin>>distance;
	cout<<"forward?: ";
	cin>>isForward;
	move(speed, distance, isForward);

	cout<<"enter angular velocity (degree/sec): ";
	cin>>angular_speed;
	cout<<"enter desired angle (degrees): ";
	cin>>angle;
	cout<<"clockwise ?: ";
	cin>>clockwise;
	rotate(degrees2radians(angular_speed), degrees2radians(angle), clockwise);
	 */
	ros::Rate loop_rate(0.5);
	/*setDesiredOrientation(degrees2radians(120));

	loop_rate.sleep();
	setDesiredOrientation(degrees2radians(-60));
	loop_rate.sleep();
	setDesiredOrientation(degrees2radians(0));*/

	/*turtlesim::Pose goal_pose;
	goal_pose.x=1;
	goal_pose.y=1;
	goal_pose.theta=0;
	moveGoal(goal_pose, 0.01);
	loop_rate.sleep();
	 */

	//gridClean();

	ros::Rate loop(0.5);
	turtlesim::Pose pose;
	pose.x=1;
	pose.y=1;
	pose.theta=0;
	moveGoal(pose, 0.01);

	pose.x=6;
	pose.y=6;
	pose.theta=0;
	moveGoal(pose, 0.01);



	loop.sleep();
	//spiralClean();

	ros::spin();

	return 0;
}

/**
 *  makes the robot move with a certain linear velocity for a
 *  certain distance in a forward or backward straight direction.
 */
void move(double speed, double distance, bool isForward){
	geometry_msgs::Twist vel_msg;
	//set a random linear velocity in the x-axis
	if (isForward)
		vel_msg.linear.x =abs(speed);
	else
		vel_msg.linear.x =-abs(speed);
	vel_msg.linear.y =0;
	vel_msg.linear.z =0;
	//set a random angular velocity in the y-axis
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z =0;

	double t0 = ros::Time::now().toSec();
	double current_distance = 0.0;
	ros::Rate loop_rate(100);
	do{
		velocity_publisher.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_distance = speed * (t1-t0);
		ros::spinOnce();
		loop_rate.sleep();
		//cout<<(t1-t0)<<", "<<current_distance <<", "<<distance<<endl;
	}while(current_distance<distance);
	vel_msg.linear.x =0;
	velocity_publisher.publish(vel_msg);

}


void rotate (double angular_speed, double relative_angle, bool clockwise){

	geometry_msgs::Twist vel_msg;
	//set a random linear velocity in the x-axis
	vel_msg.linear.x =0;
	vel_msg.linear.y =0;
	vel_msg.linear.z =0;
	//set a random angular velocity in the y-axis
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;

	if (clockwise)
		vel_msg.angular.z =-abs(angular_speed);
	else
		vel_msg.angular.z =abs(angular_speed);

	double current_angle = 0.0;
	double t0 = ros::Time::now().toSec();
	ros::Rate loop_rate(10);
	do{
		velocity_publisher.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_angle = angular_speed * (t1-t0);
		ros::spinOnce();
		loop_rate.sleep();
	}while(current_angle<relative_angle);

	vel_msg.angular.z =0;
	velocity_publisher.publish(vel_msg);

}

double degrees2radians(double angle_in_degrees){
	return angle_in_degrees *PI /180.0;
}


void setDesiredOrientation (double desired_angle_radians){
	double relative_angle_radians = desired_angle_radians - turtlesim_pose.theta;
	bool clockwise = ((relative_angle_radians<0)?true:false);
	//cout<<desired_angle_radians <<","<<turtlesim_pose.theta<<","<<relative_angle_radians<<","<<clockwise<<endl;
	rotate (degrees2radians(10), abs(relative_angle_radians), clockwise);

}

void poseCallback(const turtlesim::Pose::ConstPtr & pose_message){
	turtlesim_pose.x=pose_message->x;
	turtlesim_pose.y=pose_message->y;
	turtlesim_pose.theta=pose_message->theta;
}

double getDistance(double x1, double y1, double x2, double y2){
	return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
}

void moveGoal(turtlesim::Pose  goal_pose, double distance_tolerance){

	geometry_msgs::Twist vel_msg;

	ros::Rate loop_rate(100);
	double E = 0.0;
	do{
		/****** Proportional Controller ******/
		//linear velocity in the x-axis
		double Kp=1.0;
		double Ki=0.02;
		//double v0 = 2.0;
		//double alpha = 0.5;
		double e = getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y);
		double E = E+e;
		//Kp = v0 * (exp(-alpha)*error*error)/(error*error);
		vel_msg.linear.x = (Kp*e);
		vel_msg.linear.y =0;
		vel_msg.linear.z =0;
		//angular velocity in the z-axis
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z =4*(atan2(goal_pose.y-turtlesim_pose.y, goal_pose.x-turtlesim_pose.x)-turtlesim_pose.theta);

		velocity_publisher.publish(vel_msg);

		ros::spinOnce();
		loop_rate.sleep();

	}while(getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y)>distance_tolerance);
	cout<<"end move goal"<<endl;
	vel_msg.linear.x =0;
	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);
}



void gridClean(){

	ros::Rate loop(0.5);
	turtlesim::Pose pose;
	pose.x=1;
	pose.y=1;
	pose.theta=0;
	moveGoal(pose, 0.01);
	loop.sleep();
	setDesiredOrientation(0);
	loop.sleep();

	move(2.0, 9.0, true);
	loop.sleep();
	rotate(degrees2radians(10), degrees2radians(90), false);
	loop.sleep();
	move(2.0, 9.0, true);


	rotate(degrees2radians(10), degrees2radians(90), false);
	loop.sleep();
	move(2.0, 1.0, true);
	rotate(degrees2radians(10), degrees2radians(90), false);
	loop.sleep();
	move(2.0, 9.0, true);

	rotate(degrees2radians(30), degrees2radians(90), true);
	loop.sleep();
	move(2.0, 1.0, true);
	rotate(degrees2radians(30), degrees2radians(90), true);
	loop.sleep();
	move(2.0, 9.0, true);


	double distance = getDistance(turtlesim_pose.x, turtlesim_pose.y, x_max, y_max);

}


void spiralClean(){
	geometry_msgs::Twist vel_msg;
	double count =0;

	double constant_speed=4;
	double vk = 1;
	double wk = 2;
	double rk = 0.5;
	ros::Rate loop(1);

	do{
		rk=rk+1.0;
		vel_msg.linear.x =rk;
		vel_msg.linear.y =0;
		vel_msg.linear.z =0;
		//set a random angular velocity in the y-axis
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z =constant_speed;//((vk)/(0.5+rk));

		cout<<"vel_msg.linear.x = "<<vel_msg.linear.x<<endl;
		cout<<"vel_msg.angular.z = "<<vel_msg.angular.z<<endl;
		velocity_publisher.publish(vel_msg);
		ros::spinOnce();

		loop.sleep();
		//vk = vel_msg.linear.x;
		//wk = vel_msg.angular.z;
		//rk = vk/wk;
		cout<<rk<<", "<<vk <<", "<<wk<<endl;
	}while((turtlesim_pose.x<10.5)&&(turtlesim_pose.y<10.5));
	vel_msg.linear.x =0;
	velocity_publisher.publish(vel_msg);
}





