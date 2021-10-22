#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include "velodyne_detection/detection.h"


namespace velodyne_detection
{
  class DetectionNodelet: public nodelet::Nodelet
  {
  public:

    DetectionNodelet() {}
    ~DetectionNodelet() {}

  private:

    virtual void onInit();
    boost::shared_ptr<Detection> tf_;
  };

  /** @brief Nodelet initialization. */
  void DetectionNodelet::onInit()
  {
    tf_.reset(new Detection(getNodeHandle(), getPrivateNodeHandle()));
  }

} // namespace velodyne_pointcloud


// Register this plugin with pluginlib.  Names must match nodelets.xml.
//
// parameters: class type, base class type
PLUGINLIB_EXPORT_CLASS(velodyne_detection::DetectionNodelet, nodelet::Nodelet)