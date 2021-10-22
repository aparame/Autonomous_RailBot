#include <ros/ros.h>
#include "velodyne_detection/detection.h"


int main(int argc, char **argv) {
  // ROS init
  ros::init(argc, argv, "velodyne_detection_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  // create Detection class
  velodyne_detection::Detection detection(nh, nh_priv);

  ros::spin();
  return 0;
}