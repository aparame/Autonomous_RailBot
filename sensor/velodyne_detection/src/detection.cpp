#include <ros/ros.h>
#include "velodyne_detection/detection.h"

namespace velodyne_detection
{
  Detection::Detection(ros::NodeHandle &node, ros::NodeHandle &private_nh):
    node(node), private_nh(private_nh)
  {
    // use private node handle to get parameters
    private_nh.param("frame_id", config_.frame_id, std::string("velodyne"));
    private_nh.param("angle_offset", config_.angle_offset, 0.0);
    config_.real_angle = config_.angle_offset;
    private_nh.param("track_width", config_.track_width, 1.524);
    private_nh.param("safe_pad", config_.safe_pad, 0.1524);
    private_nh.param("ClusterTolerance", config_.ClusterTolerance, 0.3);
    private_nh.param("MinClusterSize", config_.MinClusterSize, 10);
    private_nh.param("MaxClusterSize", config_.MaxClusterSize, 600);
    private_nh.param("FieldOfView", config_.FieldOfView, 120.0);
    private_nh.param("MinHeight", config_.MinHeight, -1.2);
    private_nh.param("MaxHeight", config_.MaxHeight, 0.6);
    private_nh.param("DangerDistance", config_.DangerDistance, 3.0);

    srv_ = boost::make_shared<dynamic_reconfigure::Server<velodyne_detection::
    DetectionNodeConfig> > (private_nh);
    dynamic_reconfigure::Server<velodyne_detection::
    DetectionNodeConfig>::CallbackType f;
    f = boost::bind (&Detection::reconfigure_callback, this, _1, _2);
    srv_->setCallback (f);
    ros::Rate rate(5);
    obstacle_distances = node.advertise<std_msgs::Float32MultiArray>("obstacle_distances", 1);
    obstacle = node.advertise<std_msgs::Bool>("obstacle", 1);
    mission_forward = node.subscribe("mission_forward", 1, &Detection::mission_forward_cb, this);
    velodyne_points = node.subscribe("velodyne_points", 1, &Detection::cloud_cb, this);
  }

  Detection::trackinfo Detection::ontrack(float x, float y) {
    bool ontrack = false;
    float width = config_.safe_pad + 0.5 * config_.track_width;
    float r_angle = - config_.real_angle/180.0*3.14159;
    float x1 = x * cosf(r_angle) - y * sinf(r_angle);
    float y1 = x * sinf(r_angle) + y * cosf(r_angle);
    if (x1 > 0.0 && abs(y1) <= width) ontrack = true;
    Detection::trackinfo trackinfo;
    trackinfo.ontrack = ontrack;
    trackinfo.distance = x1;
    return trackinfo;
  }

  void Detection::inFieldOfView(pcl::PointCloud<pcl::PointXYZ>::Ptr &ros_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud) {
    // std::vector<int>::const_iterator pit;
    pcl::PointCloud<pcl::PointXYZ>::iterator pit;
    float r_angle;
    float theta = config_.real_angle/180.0*3.14159;
    for (pit = ros_cloud->begin(); pit != ros_cloud->end(); pit++) {
      float x0 = pit->x;
      float y0 = pit->y;
      float z0 = pit->z;
      r_angle = acosf( (x0*cosf(theta) + y0*sinf(theta))/sqrt(x0*x0+y0*y0));
      if (r_angle*180.0/3.14159 < 0.5*config_.FieldOfView && z0 < config_.MaxHeight && z0 >= config_.MinHeight)
        input_cloud->points.push_back(pcl::PointXYZ(pit->x, pit->y, pit->z));
    }
  }

  void Detection::mission_forward_cb(const std_msgs::Bool &forward) {
    if (forward.data) {
      config_.real_angle = config_.angle_offset;
    }
    else { // backward
      config_.real_angle = config_.angle_offset + 180.0;
    }
  }

  void Detection::cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input)

  {

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ros_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    /* Creating the KdTree from input point cloud*/
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>);

    pcl::fromROSMsg(*input, *ros_cloud);

    inFieldOfView(ros_cloud, input_cloud);

    tree->setInputCloud(input_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(config_.ClusterTolerance);
    ec.setMinClusterSize(config_.MinClusterSize);
    ec.setMaxClusterSize(config_.MaxClusterSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(input_cloud);
    ec.extract(cluster_indices);

    std::vector<pcl::PointIndices>::const_iterator it;
    std::vector<int>::const_iterator pit;
    std::vector<float> distances;
    for (it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
      float distance_min = max_range;
      for (pit = it->indices.begin(); pit != it->indices.end(); pit++) {

        float x0 = input_cloud->points[*pit].x;
        float y0 = input_cloud->points[*pit].y;
        float z0 = input_cloud->points[*pit].z;

        trackinfo local_trackinfo = ontrack(x0, y0);
        if (local_trackinfo.ontrack == true && local_trackinfo.distance < distance_min) {
          distance_min = local_trackinfo.distance;
        }

      }

      distances.push_back(distance_min);

    }

    std::vector<float> distances_pub;
    mission_stop.data = false;
    for (std::vector<float>::iterator it = distances.begin() ; it != distances.end(); ++it) {
      if (*it < config_.DangerDistance && *it >= 0.0) {
        if (distances_pub.size() < 10) distances_pub.push_back(*it);
        mission_stop.data = true;
      }
      else if (*it > 10.0){
        // distances[ii] = max_range;
      } else {

      }
    }

    std_msgs::Float32MultiArray msg;
    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim[0].size = distances_pub.size();
    msg.layout.dim[0].stride = 1;
    msg.layout.dim[0].label = "Distances";
    msg.data.insert(msg.data.end(), distances_pub.begin(), distances_pub.end());

    obstacle.publish(mission_stop);
    obstacle_distances.publish(msg);

  }

  void Detection::reconfigure_callback(
      velodyne_detection::DetectionNodeConfig &config, uint32_t level)
  {
    ROS_INFO("Reconfigure Request");
    // use private node handle to get parameters
    // config_.frame_id = config_.frame_id;
    config_.angle_offset = config.angle_offset;
    config_.track_width = config.track_width;
    config_.safe_pad = config.safe_pad;
    config_.ClusterTolerance = config.ClusterTolerance;
    config_.MinClusterSize = config.MinClusterSize;
    config_.MaxClusterSize = config.MaxClusterSize;
    config_.FieldOfView = config.FieldOfView;
    config_.MinHeight = config.MinHeight;
    config_.MaxHeight = config.MaxHeight;
    config_.DangerDistance = config.DangerDistance;
  }

}



