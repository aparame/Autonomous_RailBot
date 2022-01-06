#include "ros/ros.h"
#include "std_msgs/String.h"

void healthCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("Health [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Monitor");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("imu", 1000, healthCallback);
	ros::spin();
	return 0;
}
