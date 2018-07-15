/*
 * Author: Anis Koubaa for Gaitech EDU
 * Year: 2016
 *
 */


#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv)
{
	// Initiate new ROS node named "talker"
	ros::init(argc, argv, "talker_node");

	//create a node handle: it is reference assigned to a new node
	ros::NodeHandle n;
	//create a publisher with a topic "chatter" that will send a String message
	ros::Publisher chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
	//Rate is a class the is used to define frequency for a loop. Here we send a message each two seconds.
	ros::Rate loop_rate(0.5); //1 message per second

   int count = 0;
   while (ros::ok()) // Keep spinning loop until user presses Ctrl+C
   {
       //create a new String ROS message.
	   //Message definition in this link http://docs.ros.org/api/std_msgs/html/msg/String.html
	   std_msgs::String msg;

       //create a string for the data
	   std::stringstream ss;
	   ss << "Hello World " << count;
	   //assign the string data to ROS message data field
       msg.data = ss.str();

       //print the content of the message in the terminal
       ROS_INFO("[Talker] I published %s\n", msg.data.c_str());

       //Publish the message
       chatter_publisher.publish(msg);

       ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages

      loop_rate.sleep(); // Sleep for the rest of the cycle, to enforce the loop rate
       count++;
   }
   return 0;
}


