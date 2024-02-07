#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <sstream>

using std::cout;
using std::getline;
using std::cin;

ros::Publisher chatter_pub;

int main(int argc, char **argv)
{
	std::string username;
	cout << "Enter your username: ";
	getline(cin, username);
	ros::init(argc, argv, username);
	ros::NodeHandle n;
	
	chatter_pub = n.advertise < std_msgs::String>("chat", 1000);
	ros::Rate loop_rate(1);

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
	//spinner.stop();
	return 0;
}

