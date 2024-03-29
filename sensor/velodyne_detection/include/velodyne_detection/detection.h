#ifndef VELODYNE_DETECTION_H
#define VELODYNE_DETECTION_H

#include <string>
#include <ros/ros.h>
#include <math.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
// #include <opencv2/video/tracking.hpp>
#include <geometry_msgs/Point.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <visualization_msgs/MarkerArray.h>
#include <dynamic_reconfigure/server.h>
#include <velodyne_detection/DetectionNodeConfig.h>

namespace velodyne_detection
{

  class Detection
  {
  public:
    Detection(
        ros::NodeHandle &node,
        ros::NodeHandle &private_nh);
    ~Detection()
    {
    }

  private:

    float max_range = 999.999; // in meters
    std_msgs::Bool mission_stop;

    // Callback for dynamic reconfigure
    void reconfigure_callback(velodyne_detection::DetectionNodeConfig &config,
              uint32_t level);
    // Pointer to dynamic reconfigure service srv_
    boost::shared_ptr<dynamic_reconfigure::Server<velodyne_detection::DetectionNodeConfig> > srv_;

    typedef struct
    {
      bool ontrack;
      float distance;
    }
    trackinfo;

    trackinfo ontrack(float x, float y);
    void inFieldOfView(pcl::PointCloud<pcl::PointXYZ>::Ptr &ros_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud);

    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input);

    void mission_forward_cb(const std_msgs::Bool &forward);

    ros::NodeHandle node;
    ros::NodeHandle private_nh;
    ros::Subscriber velodyne_points;
    ros::Subscriber mission_forward;
    ros::Publisher obstacle;
    ros::Publisher obstacle_distances;

    /// configuration parameters
    typedef struct
    {
      std::string frame_id;      ///< tf frame ID
      double angle_offset;       ///< difference between lidar 0' direction and track direction in degrees
      double real_angle;
      double track_width;        ///< width of track in meters
      double safe_pad;         ///< safe padding width in meters
      double ClusterTolerance;
      int MinClusterSize;
      int MaxClusterSize;
      double FieldOfView; // in degrees
      double MinHeight; // relative to the Lidar in meters
      double MaxHeight;
      double DangerDistance;
    }
    Config;
    Config config_;

  };
}  // namespace velodyne_detection

#endif
