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
#include <opencv2/video/tracking.hpp>
#include <geometry_msgs/Point.h>

#include <pcl/segmentation/extract_clusters.h>
// #include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <sensor_msgs/PointCloud2.h>

// #include <limits>
// #include <utility>
// #include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <dynamic_reconfigure/server.h>
#include <velodyne_detection/DetectionNodeConfig.h>

namespace velodyne_detection
{
  // using DetectionNodeCfg = velodyne_pointcloud::DetectionNodeConfig;

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

    ros::Publisher pub_cluster0;
    ros::Publisher pub_cluster1;
    ros::Publisher pub_cluster2;
    ros::Publisher pub_cluster3;
    ros::Publisher pub_cluster4;
    ros::Publisher pub_cluster5;

    ros::Publisher objID_pub;
    ros::Publisher markerPub;

    cv::KalmanFilter KF0, KF1, KF2, KF3, KF4, KF5;

    std::vector<geometry_msgs::Point> prevClusterCenters;

    float max_range = 999.999; // in meters
    std::array<float, 6> distances;
    std_msgs::Bool mission_stop;

    // Callback for dynamic reconfigure
    void reconfigure_callback(velodyne_detection::DetectionNodeConfig &config,
              uint32_t level);
    // Pointer to dynamic reconfigure service srv_
    boost::shared_ptr<dynamic_reconfigure::Server<velodyne_detection::DetectionNodeConfig> > srv_;

    void publish_cloud(ros::Publisher &pub,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr cluster);

    double euclidean_distance(geometry_msgs::Point &p1, geometry_msgs::Point &p2);

    typedef struct
    {
      bool ontrack;
      float distance;
    }
    trackinfo;

    trackinfo ontrack(float x, float y);

    std::pair<int, int> findIndexOfMin(std::vector<std::vector<float>> distMat);

    void KFT(const std_msgs::Float32MultiArray ccs);

    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input);

    // Pointer to dynamic reconfigure service srv_
    // boost::shared_ptr<dynamic_reconfigure::Server<velodyne_pointcloud::DetectionNodeConfig>> srv_;
    // void reconfigure_callback(velodyne_pointcloud::DetectionNodeConfig& config, uint32_t level);

    // boost::shared_ptr<velodyne_rawdata::RawData> data_;
    ros::NodeHandle node;
    ros::NodeHandle private_nh;
    ros::Subscriber velodyne_points;
    ros::Publisher obstacle;
    ros::Publisher obstacle_distances;

    /// configuration parameters
    typedef struct
    {
      std::string frame_id;      ///< tf frame ID
      double angle_offset;       ///< difference between lidar 0' direction and track direction in radians
      double track_width;        ///< width of track in meters
      double safe_pad;         ///< safe padding width in meters
    }
    Config;
    Config config_;

    bool firstFrame;
    std::vector<int> objID; // Output of the data association using KF

    // boost::shared_ptr<velodyne_rawdata::DataContainerBase> container_ptr;

  };
}  // namespace velodyne_detection

#endif
