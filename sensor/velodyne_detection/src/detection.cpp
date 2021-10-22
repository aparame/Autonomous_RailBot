// #include <algorithm>
// #include <fstream>
// #include <geometry_msgs/Point.h>
// #include <iostream>
// #include <iterator>
// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/video/video.hpp>
// #include <pcl/io/pcd_io.h>
#include <ros/ros.h>
// #include <string.h>

// #include <vector>
// #include <string>
// #include <opencv2/video/tracking.hpp>

// #include <pcl/common/centroid.h>
// #include <pcl/common/geometry.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/filters/extract_indices.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/kdtree/kdtree.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/sample_consensus/model_types.h>
// #include <pcl/segmentation/extract_clusters.h>
// #include <pcl/segmentation/sac_segmentation.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <sensor_msgs/PointCloud2.h>

// #include <limits>
// #include <utility>
// #include <visualization_msgs/Marker.h>
// #include <visualization_msgs/MarkerArray.h>
#include "velodyne_detection/detection.h"
// using namespace std;
// using namespace cv;

namespace velodyne_detection
{
  Detection::Detection(ros::NodeHandle &node, ros::NodeHandle &private_nh):
    node(node), private_nh(private_nh), firstFrame(true)
  {
    // use private node handle to get parameters
    private_nh.param("frame_id", config_.frame_id, std::string("velodyne"));
    private_nh.param("angle_offset", config_.angle_offset, 0.0);
    private_nh.param("track_width", config_.track_width, 1.524);
    private_nh.param("safe_pad", config_.safe_pad, 0.1524);
    private_nh.param("ClusterTolerance", config_.ClusterTolerance, 0.3);
    private_nh.param("MinClusterSize", config_.MinClusterSize, 10);
    private_nh.param("MaxClusterSize", config_.MaxClusterSize, 600);
    private_nh.param("FieldOfView", config_.FieldOfView, 120.0);

    srv_ = boost::make_shared<dynamic_reconfigure::Server<velodyne_detection::
    DetectionNodeConfig> > (private_nh);
    dynamic_reconfigure::Server<velodyne_detection::
    DetectionNodeConfig>::CallbackType f;
    f = boost::bind (&Detection::reconfigure_callback, this, _1, _2);
    srv_->setCallback (f);

    // KF init
    int stateDim = 4; // [x,y,v_x,v_y]//,w,h]
    int measDim = 2;  // [z_x,z_y,z_w,z_h]
    int ctrlDim = 0;

    KF0.init(stateDim, measDim, ctrlDim, CV_32F);
    KF1.init(stateDim, measDim, ctrlDim, CV_32F);
    KF2.init(stateDim, measDim, ctrlDim, CV_32F);
    KF3.init(stateDim, measDim, ctrlDim, CV_32F);
    KF4.init(stateDim, measDim, ctrlDim, CV_32F);
    KF5.init(stateDim, measDim, ctrlDim, CV_32F);

    cv::Mat state(stateDim, 1, CV_32F);
    cv::Mat_<float> measurement(2, 1);

    // Create a ROS publisher for the output point cloud
    pub_cluster0 = node.advertise<sensor_msgs::PointCloud2>("obstacles/cluster_0", 1);
    pub_cluster1 = node.advertise<sensor_msgs::PointCloud2>("obstacles/cluster_1", 1);
    pub_cluster2 = node.advertise<sensor_msgs::PointCloud2>("obstacles/cluster_2", 1);
    pub_cluster3 = node.advertise<sensor_msgs::PointCloud2>("obstacles/cluster_3", 1);
    pub_cluster4 = node.advertise<sensor_msgs::PointCloud2>("obstacles/cluster_4", 1);
    pub_cluster5 = node.advertise<sensor_msgs::PointCloud2>("obstacles/cluster_5", 1);
    // Subscribe to the clustered pointclouds
    // ros::Subscriber c1=nh.subscribe("ccs",100,KFT);
    objID_pub = node.advertise<std_msgs::Int32MultiArray>("obstacles/obj_id", 1);
    /* Point cloud clustering
    */
    // cc_pos=nh.advertise<std_msgs::Float32MultiArray>("ccs",100);//clusterCenter1
    markerPub = node.advertise<visualization_msgs::MarkerArray>("obstacles/viz", 1);

    obstacle = node.advertise<std_msgs::Bool>("obstacle", 1);

    obstacle_distances = node.advertise<std_msgs::Float32MultiArray>("obstacle_distances", 1);

    velodyne_points = node.subscribe("velodyne_points", 1, &Detection::cloud_cb, this);
  }

  Detection::trackinfo Detection::ontrack(float x, float y) {
    bool ontrack = false;
    float width = config_.safe_pad + 0.5 * config_.track_width;
    float r_angle = - config_.angle_offset/180.0*3.14159;
    float x1 = x * cosf(r_angle) - y * sinf(r_angle);
    float y1 = x * sinf(r_angle) + y * cosf(r_angle);
    if (x1 > 0.0 && abs(y1) <= width) ontrack = true;
    Detection::trackinfo trackinfo;
    trackinfo.ontrack = ontrack;
    trackinfo.distance = x1;
    return trackinfo;
  }

  void Detection::inFieldOfView(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud) {
    // std::vector<int>::const_iterator pit;
    pcl::PointCloud<pcl::PointXYZ>::iterator pit;
    float r_angle;
    float theta = config_.angle_offset/180.0*3.14159;
    for (pit = input_cloud->begin(); pit != input_cloud->end(); pit++) {
      float x0 = pit->x;
      float y0 = pit->y;
      r_angle = acosf( (x0*cosf(theta) + y0*sinf(theta))/sqrt(x0*x0+y0*y0));
      if (abs(r_angle) > 0.5*config_.FieldOfView)
        input_cloud->erase(pit);
    }
  }

  // calculate euclidean distance of two points
  double Detection::euclidean_distance(geometry_msgs::Point &p1, geometry_msgs::Point &p2) {
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) +
                (p1.z - p2.z) * (p1.z - p2.z));
  }

  /*
  objID: vector containing the IDs of the clusters that should be associated with
  each KF_Tracker objID[0] corresponds to KFT0, objID[1] corresponds to KFT1 etc.
  */

  std::pair<int, int> Detection::findIndexOfMin(std::vector<std::vector<float>> distMat) {
    // cout << "findIndexOfMin cALLED\n";
    std::pair<int, int> minIndex;
    float minEl = std::numeric_limits<float>::max();
    // cout << "minEl=" << minEl << "\n";
    for (int i = 0; i < distMat.size(); i++)
      for (int j = 0; j < distMat.at(0).size(); j++) {
        if (distMat[i][j] < minEl) {
          minEl = distMat[i][j];
          minIndex = std::make_pair(i, j);
        }
      }
    // cout << "minIndex=" << minIndex.first << "," << minIndex.second << "\n";
    return minIndex;
  }

  void Detection::KFT(const std_msgs::Float32MultiArray ccs) {

    // First predict, to update the internal statePre variable

    std::vector<cv::Mat> pred{KF0.predict(), KF1.predict(), KF2.predict(),
                              KF3.predict(), KF4.predict(), KF5.predict()};

    // Get measurements
    // Extract the position of the clusters forom the multiArray. To check if the
    // data coming in, check the .z (every third) coordinate and that will be 0.0
    std::vector<geometry_msgs::Point> clusterCenters; // clusterCenters

    int i = 0;
    for (std::vector<float>::const_iterator it = ccs.data.begin();
        it != ccs.data.end(); it += 3) {
      geometry_msgs::Point pt;
      pt.x = *it;
      pt.y = *(it + 1);
      pt.z = *(it + 2);

      clusterCenters.push_back(pt);
    }

    std::vector<geometry_msgs::Point> KFpredictions;
    i = 0;
    for (auto it = pred.begin(); it != pred.end(); it++) {
      geometry_msgs::Point pt;
      pt.x = (*it).at<float>(0);
      pt.y = (*it).at<float>(1);
      pt.z = (*it).at<float>(2);

      KFpredictions.push_back(pt);
    }

    // Find the cluster that is more probable to be belonging to a given KF.
    objID.clear();   // Clear the objID vector
    objID.resize(6); // Allocate default elements so that [i] doesnt segfault.
                    // Should be done better
    // Copy clusterCentres for modifying it and preventing multiple assignments of
    // the same ID
    std::vector<geometry_msgs::Point> copyOfClusterCenters(clusterCenters);
    std::vector<std::vector<float>> distMat;

    for (int filterN = 0; filterN < 6; filterN++) {
      std::vector<float> distVec;
      for (int n = 0; n < 6; n++) {
        distVec.push_back(
            euclidean_distance(KFpredictions[filterN], copyOfClusterCenters[n]));
      }

      distMat.push_back(distVec);

    }

  /*
    cout << "distMat.size()" << distMat.size() << "\n";
    cout << "distMat[0].size()" << distMat.at(0).size() << "\n";

    // DEBUG: print the distMat
    for (const auto &row : distMat) {
      for (const auto &s : row)
        std::cout << s << ' ';
      std::cout << std::endl;
    }
  */

    for (int clusterCount = 0; clusterCount < 6; clusterCount++) {
      // 1. Find min(distMax)==> (i,j);
      std::pair<int, int> minIndex(findIndexOfMin(distMat));
      // cout << "Received minIndex=" << minIndex.first << "," << minIndex.second << "\n";
      // 2. objID[i]=clusterCenters[j]; counter++
      objID[minIndex.first] = minIndex.second;

      // 3. distMat[i,:]=10000; distMat[:,j]=10000
      distMat[minIndex.first] =
          std::vector<float>(6, 10000.0); // Set the row to a high number.
      for (int row = 0; row < distMat.size();
          row++) // set the column to a high number
      {
        distMat[row][minIndex.second] = 10000.0;
      }
      // 4. if(counter<6) got to 1.
      // cout << "clusterCount=" << clusterCount << "\n";
    }

    // cout<<"Got object IDs"<<"\n";
    // countIDs(objID);// for verif/corner cases

    // display objIDs
    /* DEBUG
      cout<<"objID= ";
      for(auto it=objID.begin();it!=objID.end();it++)
          cout<<*it<<" ,";
      cout<<"\n";
      */

    visualization_msgs::MarkerArray clusterMarkers;

    for (int i = 0; i < 6; i++) {
      visualization_msgs::Marker m;

      m.id = i;
      m.type = visualization_msgs::Marker::CUBE;
      m.header.frame_id = config_.frame_id;
      m.scale.x = 0.3;
      m.scale.y = 0.3;
      m.scale.z = 0.3;
      m.action = visualization_msgs::Marker::ADD;
      m.color.a = 1.0;
      m.color.r = i % 2 ? 1 : 0;
      m.color.g = i % 3 ? 1 : 0;
      m.color.b = i % 4 ? 1 : 0;

      // geometry_msgs::Point clusterC(clusterCenters.at(objID[i]));
      geometry_msgs::Point clusterC(KFpredictions[i]);
      m.pose.position.x = clusterC.x;
      m.pose.position.y = clusterC.y;
      m.pose.position.z = clusterC.z;

      clusterMarkers.markers.push_back(m);
    }

    prevClusterCenters = clusterCenters;

    markerPub.publish(clusterMarkers);

    std_msgs::Int32MultiArray obj_id;
    for (auto it = objID.begin(); it != objID.end(); it++)
      obj_id.data.push_back(*it);
    // Publish the object IDs
    objID_pub.publish(obj_id);
    // convert clusterCenters from geometry_msgs::Point to floats
    std::vector<std::vector<float>> cc;
    for (int i = 0; i < 6; i++) {
      std::vector<float> pt;
      pt.push_back(clusterCenters[objID[i]].x);
      pt.push_back(clusterCenters[objID[i]].y);
      pt.push_back(clusterCenters[objID[i]].z);

      cc.push_back(pt);
    }
    // cout<<"cc[5][0]="<<cc[5].at(0)<<"cc[5][1]="<<cc[5].at(1)<<"cc[5][2]="<<cc[5].at(2)<<"\n";
    float meas0[2] = {cc[0].at(0), cc[0].at(1)};
    float meas1[2] = {cc[1].at(0), cc[1].at(1)};
    float meas2[2] = {cc[2].at(0), cc[2].at(1)};
    float meas3[2] = {cc[3].at(0), cc[3].at(1)};
    float meas4[2] = {cc[4].at(0), cc[4].at(1)};
    float meas5[2] = {cc[5].at(0), cc[5].at(1)};

    // The update phase
    cv::Mat meas0Mat = cv::Mat(2, 1, CV_32F, meas0);
    cv::Mat meas1Mat = cv::Mat(2, 1, CV_32F, meas1);
    cv::Mat meas2Mat = cv::Mat(2, 1, CV_32F, meas2);
    cv::Mat meas3Mat = cv::Mat(2, 1, CV_32F, meas3);
    cv::Mat meas4Mat = cv::Mat(2, 1, CV_32F, meas4);
    cv::Mat meas5Mat = cv::Mat(2, 1, CV_32F, meas5);

    // cout<<"meas0Mat"<<meas0Mat<<"\n";
    if (!(meas0Mat.at<float>(0, 0) == 0.0f || meas0Mat.at<float>(1, 0) == 0.0f))
      cv::Mat estimated0 = KF0.correct(meas0Mat);
    if (!(meas1[0] == 0.0f || meas1[1] == 0.0f))
      cv::Mat estimated1 = KF1.correct(meas1Mat);
    if (!(meas2[0] == 0.0f || meas2[1] == 0.0f))
      cv::Mat estimated2 = KF2.correct(meas2Mat);
    if (!(meas3[0] == 0.0f || meas3[1] == 0.0f))
      cv::Mat estimated3 = KF3.correct(meas3Mat);
    if (!(meas4[0] == 0.0f || meas4[1] == 0.0f))
      cv::Mat estimated4 = KF4.correct(meas4Mat);
    if (!(meas5[0] == 0.0f || meas5[1] == 0.0f))
      cv::Mat estimated5 = KF5.correct(meas5Mat);

    // Publish the point clouds belonging to each clusters

    // cout<<"estimate="<<estimated.at<float>(0)<<","<<estimated.at<float>(1)<<"\n";
    // Point statePt(estimated.at<float>(0),estimated.at<float>(1));
    // cout<<"DONE KF_TRACKER\n";
  }

  void Detection::publish_cloud(ros::Publisher &pub,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster) {
    sensor_msgs::PointCloud2::Ptr clustermsg(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*cluster, *clustermsg);
    clustermsg->header.frame_id = config_.frame_id;
    clustermsg->header.stamp = ros::Time::now();
    pub.publish(*clustermsg);
  }

  void Detection::cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input)

  {
    // cout<<"IF firstFrame="<<firstFrame<<"\n";
    // If this is the first frame, initialize kalman filters for the clustered
    // objects
    if (firstFrame) {
      // Initialize 6 Kalman Filters; Assuming 6 max objects in the dataset.
      // Could be made generic by creating a Kalman Filter only when a new object
      // is detected

      float dvx = 0.01f; // 1.0
      float dvy = 0.01f; // 1.0
      float dx = 1.0f;
      float dy = 1.0f;
      KF0.transitionMatrix = (cv::Mat_<float>(4, 4) << dx, 0, 1, 0, 0, dy, 0, 1, 0, 0,
                              dvx, 0, 0, 0, 0, dvy);
      KF1.transitionMatrix = (cv::Mat_<float>(4, 4) << dx, 0, 1, 0, 0, dy, 0, 1, 0, 0,
                              dvx, 0, 0, 0, 0, dvy);
      KF2.transitionMatrix = (cv::Mat_<float>(4, 4) << dx, 0, 1, 0, 0, dy, 0, 1, 0, 0,
                              dvx, 0, 0, 0, 0, dvy);
      KF3.transitionMatrix = (cv::Mat_<float>(4, 4) << dx, 0, 1, 0, 0, dy, 0, 1, 0, 0,
                              dvx, 0, 0, 0, 0, dvy);
      KF4.transitionMatrix = (cv::Mat_<float>(4, 4) << dx, 0, 1, 0, 0, dy, 0, 1, 0, 0,
                              dvx, 0, 0, 0, 0, dvy);
      KF5.transitionMatrix = (cv::Mat_<float>(4, 4) << dx, 0, 1, 0, 0, dy, 0, 1, 0, 0,
                              dvx, 0, 0, 0, 0, dvy);

      cv::setIdentity(KF0.measurementMatrix);
      cv::setIdentity(KF1.measurementMatrix);
      cv::setIdentity(KF2.measurementMatrix);
      cv::setIdentity(KF3.measurementMatrix);
      cv::setIdentity(KF4.measurementMatrix);
      cv::setIdentity(KF5.measurementMatrix);
      // Process Noise Covariance Matrix Q
      // [ Ex 0  0    0 0    0 ]
      // [ 0  Ey 0    0 0    0 ]
      // [ 0  0  Ev_x 0 0    0 ]
      // [ 0  0  0    1 Ev_y 0 ]
      //// [ 0  0  0    0 1    Ew ]
      //// [ 0  0  0    0 0    Eh ]
      float sigmaP = 0.01;
      float sigmaQ = 0.1;
      cv::setIdentity(KF0.processNoiseCov, cv::Scalar::all(sigmaP));
      cv::setIdentity(KF1.processNoiseCov, cv::Scalar::all(sigmaP));
      cv::setIdentity(KF2.processNoiseCov, cv::Scalar::all(sigmaP));
      cv::setIdentity(KF3.processNoiseCov, cv::Scalar::all(sigmaP));
      cv::setIdentity(KF4.processNoiseCov, cv::Scalar::all(sigmaP));
      cv::setIdentity(KF5.processNoiseCov, cv::Scalar::all(sigmaP));
      // Meas noise cov matrix R
      cv::setIdentity(KF0.measurementNoiseCov, cv::Scalar(sigmaQ)); // 1e-1
      cv::setIdentity(KF1.measurementNoiseCov, cv::Scalar(sigmaQ));
      cv::setIdentity(KF2.measurementNoiseCov, cv::Scalar(sigmaQ));
      cv::setIdentity(KF3.measurementNoiseCov, cv::Scalar(sigmaQ));
      cv::setIdentity(KF4.measurementNoiseCov, cv::Scalar(sigmaQ));
      cv::setIdentity(KF5.measurementNoiseCov, cv::Scalar(sigmaQ));

      // Process the point cloud
      pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(
          new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud(
          new pcl::PointCloud<pcl::PointXYZ>);
      /* Creating the KdTree from input point cloud*/
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
          new pcl::search::KdTree<pcl::PointXYZ>);

      pcl::fromROSMsg(*input, *input_cloud);
      inFieldOfView(input_cloud);

      tree->setInputCloud(input_cloud);

      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
      ec.setClusterTolerance(config_.ClusterTolerance);
      ec.setMinClusterSize(config_.MinClusterSize);
      ec.setMaxClusterSize(config_.MaxClusterSize);
      ec.setSearchMethod(tree);
      ec.setInputCloud(input_cloud);
      /* Extract the clusters out of pc and save indices in cluster_indices.*/
      ec.extract(cluster_indices);

      std::vector<pcl::PointIndices>::const_iterator it;
      std::vector<int>::const_iterator pit;
      // Vector of cluster pointclouds
      std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_vec;
      // Cluster centroids
      std::vector<pcl::PointXYZ> clusterCentroids;

      for (it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(
            new pcl::PointCloud<pcl::PointXYZ>);
        float x = 0.0;
        float y = 0.0;
        int numPts = 0;
        for (pit = it->indices.begin(); pit != it->indices.end(); pit++) {

          cloud_cluster->points.push_back(input_cloud->points[*pit]);
          x += input_cloud->points[*pit].x;
          y += input_cloud->points[*pit].y;
          numPts++;

          // dist_this_point = pcl::geometry::distance(input_cloud->points[*pit],
          //                                          origin);
          // mindist_this_cluster = std::min(dist_this_point,
          // mindist_this_cluster);
        }

        pcl::PointXYZ centroid;
        centroid.x = x / numPts;
        centroid.y = y / numPts;
        centroid.z = 0.0;

        cluster_vec.push_back(cloud_cluster);

        // Get the centroid of the cluster
        clusterCentroids.push_back(centroid);
      }

      // Ensure at least 6 clusters exist to publish (later clusters may be empty)
      while (cluster_vec.size() < 6) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr empty_cluster(
            new pcl::PointCloud<pcl::PointXYZ>);
        empty_cluster->points.push_back(pcl::PointXYZ(0, 0, 0));
        cluster_vec.push_back(empty_cluster);
      }

      while (clusterCentroids.size() < 6) {
        pcl::PointXYZ centroid;
        centroid.x = 0.0;
        centroid.y = 0.0;
        centroid.z = 0.0;

        clusterCentroids.push_back(centroid);
      }

      // Set initial state
      KF0.statePre.at<float>(0) = clusterCentroids.at(0).x;
      KF0.statePre.at<float>(1) = clusterCentroids.at(0).y;
      KF0.statePre.at<float>(2) = 0; // initial v_x
      KF0.statePre.at<float>(3) = 0; // initial v_y

      // Set initial state
      KF1.statePre.at<float>(0) = clusterCentroids.at(1).x;
      KF1.statePre.at<float>(1) = clusterCentroids.at(1).y;
      KF1.statePre.at<float>(2) = 0; // initial v_x
      KF1.statePre.at<float>(3) = 0; // initial v_y

      // Set initial state
      KF2.statePre.at<float>(0) = clusterCentroids.at(2).x;
      KF2.statePre.at<float>(1) = clusterCentroids.at(2).y;
      KF2.statePre.at<float>(2) = 0; // initial v_x
      KF2.statePre.at<float>(3) = 0; // initial v_y

      // Set initial state
      KF3.statePre.at<float>(0) = clusterCentroids.at(3).x;
      KF3.statePre.at<float>(1) = clusterCentroids.at(3).y;
      KF3.statePre.at<float>(2) = 0; // initial v_x
      KF3.statePre.at<float>(3) = 0; // initial v_y

      // Set initial state
      KF4.statePre.at<float>(0) = clusterCentroids.at(4).x;
      KF4.statePre.at<float>(1) = clusterCentroids.at(4).y;
      KF4.statePre.at<float>(2) = 0; // initial v_x
      KF4.statePre.at<float>(3) = 0; // initial v_y

      // Set initial state
      KF5.statePre.at<float>(0) = clusterCentroids.at(5).x;
      KF5.statePre.at<float>(1) = clusterCentroids.at(5).y;
      KF5.statePre.at<float>(2) = 0; // initial v_x
      KF5.statePre.at<float>(3) = 0; // initial v_y

      firstFrame = false;

      for (int i = 0; i < 6; i++) {
        geometry_msgs::Point pt;
        pt.x = clusterCentroids.at(i).x;
        pt.y = clusterCentroids.at(i).y;
        prevClusterCenters.push_back(pt);
      }
      /*  // Print the initial state of the kalman filter for debugging
        cout<<"KF0.satePre="<<KF0.statePre.at<float>(0)<<","<<KF0.statePre.at<float>(1)<<"\n";
        cout<<"KF1.satePre="<<KF1.statePre.at<float>(0)<<","<<KF1.statePre.at<float>(1)<<"\n";
        cout<<"KF2.satePre="<<KF2.statePre.at<float>(0)<<","<<KF2.statePre.at<float>(1)<<"\n";
        cout<<"KF3.satePre="<<KF3.statePre.at<float>(0)<<","<<KF3.statePre.at<float>(1)<<"\n";
        cout<<"KF4.satePre="<<KF4.statePre.at<float>(0)<<","<<KF4.statePre.at<float>(1)<<"\n";
        cout<<"KF5.satePre="<<KF5.statePre.at<float>(0)<<","<<KF5.statePre.at<float>(1)<<"\n";

        //cin.ignore();// To be able to see the printed initial state of the
        KalmanFilter
        */
    }

    else {
      // cout<<"ELSE firstFrame="<<firstFrame<<"\n";
      pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(
          new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud(
          new pcl::PointCloud<pcl::PointXYZ>);
      /* Creating the KdTree from input point cloud*/
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
          new pcl::search::KdTree<pcl::PointXYZ>);

      pcl::fromROSMsg(*input, *input_cloud);
      inFieldOfView(input_cloud);

      tree->setInputCloud(input_cloud);

      /* Here we are creating a vector of PointIndices, which contains the actual
      * index information in a vector<int>. The indices of each detected cluster
      * are saved here. Cluster_indices is a vector containing one instance of
      * PointIndices for each detected cluster. Cluster_indices[0] contain all
      * indices of the first cluster in input point cloud.
      */
      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
      ec.setClusterTolerance(config_.ClusterTolerance);
      ec.setMinClusterSize(config_.MinClusterSize);
      ec.setMaxClusterSize(config_.MaxClusterSize);
      ec.setSearchMethod(tree);
      ec.setInputCloud(input_cloud);
      // cout<<"PCL init successfull\n";
      /* Extract the clusters out of pc and save indices in cluster_indices.*/
      ec.extract(cluster_indices);
      // cout<<"PCL extract successfull\n";
      /* To separate each cluster out of the vector<PointIndices> we have to
      * iterate through cluster_indices, create a new PointCloud for each
      * entry and write all points of the current cluster in the PointCloud.
      */
      // pcl::PointXYZ origin (0,0,0);
      // float mindist_this_cluster = 1000;
      // float dist_this_point = 1000;

      std::vector<pcl::PointIndices>::const_iterator it;
      std::vector<int>::const_iterator pit;
      // Vector of cluster pointclouds
      std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_vec;

      // Cluster centroids
      std::vector<pcl::PointXYZ> clusterCentroids;

      distances = {max_range, max_range, max_range, max_range, max_range, max_range};
      int index = 0;
      for (it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
        float x = 0.0;
        float y = 0.0;
        int numPts = 0;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(
            new pcl::PointCloud<pcl::PointXYZ>);
        float distance_min = max_range;
        for (pit = it->indices.begin(); pit != it->indices.end(); pit++) {

          cloud_cluster->points.push_back(input_cloud->points[*pit]);

          float x0 = input_cloud->points[*pit].x;
          float y0 = input_cloud->points[*pit].y;
          x += x0;
          y += y0;
          numPts++;

          trackinfo local_trackinfo = ontrack(x0, y0);
          if (local_trackinfo.ontrack == true && local_trackinfo.distance < distance_min) {
            distance_min = local_trackinfo.distance;
          }
          // dist_this_point = pcl::geometry::distance(input_cloud->points[*pit],
          //                                          origin);
          // mindist_this_cluster = std::min(dist_this_point,
          // mindist_this_cluster);
        }

        distances[index] = distance_min;
        index++;

        pcl::PointXYZ centroid;
        centroid.x = x / numPts;
        centroid.y = y / numPts;
        centroid.z = 0.0;

        cluster_vec.push_back(cloud_cluster);

        // Get the centroid of the cluster
        clusterCentroids.push_back(centroid);
      }

      mission_stop.data = false;
      for (int ii=0; ii < 6; ii++){
        if (distances[ii] < 3.0 && distances[ii] >= 0.0) {
          mission_stop.data = true;
        }
        else if (distances[ii] > 10.0){
          // distances[ii] = max_range;
        } else {

        }
      }
      std_msgs::Float32MultiArray msg;
      msg.data = {distances[0],distances[1],distances[2],distances[3],distances[4],distances[5]};
      obstacle.publish(mission_stop);
      obstacle_distances.publish(msg);

      // cout<<"cluster_vec got some clusters\n";

      // Ensure at least 6 clusters exist to publish (later clusters may be empty)
      while (cluster_vec.size() < 6) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr empty_cluster(
            new pcl::PointCloud<pcl::PointXYZ>);
        empty_cluster->points.push_back(pcl::PointXYZ(0, 0, 0));
        cluster_vec.push_back(empty_cluster);
      }

      while (clusterCentroids.size() < 6) {
        pcl::PointXYZ centroid;
        centroid.x = 0.0;
        centroid.y = 0.0;
        centroid.z = 0.0;

        clusterCentroids.push_back(centroid);
      }

      std_msgs::Float32MultiArray cc;
      for (int i = 0; i < 6; i++) {
        cc.data.push_back(clusterCentroids.at(i).x);
        cc.data.push_back(clusterCentroids.at(i).y);
        cc.data.push_back(clusterCentroids.at(i).z);
      }
      // cout<<"6 clusters initialized\n";

      // cc_pos.publish(cc);// Publish cluster mid-points.
      KFT(cc);
      int i = 0;
      bool publishedCluster[6];
      for (auto it = objID.begin(); it != objID.end();
          it++) { // cout<<"Inside the for loop\n";

        switch (i) {
          case 0: {
            publish_cloud(pub_cluster0, cluster_vec[*it]);
            publishedCluster[i] =
                true; // Use this flag to publish only once for a given obj ID
            i++;
            break;
          }
          case 1: {
            publish_cloud(pub_cluster1, cluster_vec[*it]);
            publishedCluster[i] =
                true; // Use this flag to publish only once for a given obj ID
            i++;
            break;
          }
          case 2: {
            publish_cloud(pub_cluster2, cluster_vec[*it]);
            publishedCluster[i] =
                true; // Use this flag to publish only once for a given obj ID
            i++;
            break;
          }
          case 3: {
            publish_cloud(pub_cluster3, cluster_vec[*it]);
            publishedCluster[i] =
                true; // Use this flag to publish only once for a given obj ID
            i++;
            break;
          }
          case 4: {
            publish_cloud(pub_cluster4, cluster_vec[*it]);
            publishedCluster[i] =
                true; // Use this flag to publish only once for a given obj ID
            i++;
            break;
          }
          case 5: {
            publish_cloud(pub_cluster5, cluster_vec[*it]);
            publishedCluster[i] =
                true; // Use this flag to publish only once for a given obj ID
            i++;
            break;
          }
          default:
            break;
        }
      }


    }

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

  }

}



