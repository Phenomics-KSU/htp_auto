#include "ros/ros.h"

int main(int argc, char **argv)
{
	// Setup ROS system for this node.
	ros::init(argc, argv, "hello_ros");
	
	// Establish this program as a ROS node by creating handle. This is what actually connects to master.
	ros::NodeHandle nh;
	
	ROS_INFO_STREAM("Hello ROS!");
	
	return 0;
}