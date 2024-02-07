#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <sstream>

using std::cout;
using std::getline;
using std::cin;

ros::Subscriber chatter_sub;
ros::Publisher chatter_pub;

void writeMsgToLog(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("%s", msg->data.c_str());
}

int main(int argc, char **argv)
{
	std::string username;
	cout << "Enter your username: ";
	getline(cin, username);
	ros::init(argc, argv, username);
	ros::NodeHandle n;
	
	ros::AsyncSpinner spinner(1);
	spinner.start();
	
	chatter_sub = n.subscribe("chat", 1000, writeMsgToLog);
	while (ros::ok())
	{
		std_msgs::String msg;
		std::stringstream ss;
		std::string message;
		cout << username << ":";
		getline(std::cin, message);
		ss <<  username << ": "  << msg ;
		msg.data = ss.str();
		chatter_pub.publish(msg);
		ros::spinOnce();
	}
	spinner.stop();
	return 0;
}

