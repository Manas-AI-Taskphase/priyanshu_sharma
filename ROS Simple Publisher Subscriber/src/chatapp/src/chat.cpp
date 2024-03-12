#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <sstream>

using std::cout;
using std::getline;
using std::cin;

ros::Subscriber chatter_sub;
ros::Publisher chatter_pub;

std::string username;

void writeMsgToLog(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("%s", msg->data.c_str());
}
int main(int argc, char **argv)
{
	cout << "Enter your username: ";
	getline(cin, username);
	ros::init(argc, argv, username);
	ros::NodeHandle n;
	
	ros::AsyncSpinner spinner(1);
	spinner.start();
	chatter_pub = n.advertise <std_msgs::String>("chat", 1000);
	while (ros::ok())
	{
		ros::spinOnce();
		std_msgs::String msg;
		std::string message;
		chatter_sub = n.subscribe("chat", 1000, writeMsgToLog);
		cout << username << ":";
		getline(std::cin, message);
		msg.data = username + ": " + message; 
		chatter_pub.publish(msg);
	}
	spinner.stop();
	return 0;
}

