#ifndef TANGO_ROS_NODE_H_
#define TANGO_ROS_NODE_H_
#include <jni.h>
#include <string>

#include <tango_client_api.h>
#include <tango_support_api.h>

#include <opencv2/core/core.hpp>

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>

namespace tango_ros_node {
static int kNumberOfFieldsInPointCloud = 4;
static int kImageCompressingQuality = 50;

enum CameraType {
  NONE,
  FISHEYE,
  COLOR
};

struct PublisherConfiguration {
  bool publish_device_pose = false;
  bool publish_point_cloud = false;
  CameraType publish_camera = NONE;

  std::string point_cloud_topic = "tango/point_cloud";
  std::string camera_topic = "tango/image_raw/compressed";
};

class TangoRosNode {
 public:
  TangoRosNode();
  ~TangoRosNode();
  void CheckTangoVersion(JNIEnv* env, jobject activity);
  void OnTangoServiceConnected(JNIEnv* env, jobject binder);
  void TangoDisconnect();
  void Publish();

  void OnPoseAvailable(const TangoPoseData* pose);
  void OnPointCloudAvailable(const TangoPointCloud* point_cloud);
  void OnFrameAvailable(const TangoImageBuffer* buffer);

 private:
  bool TangoSetupConfig();
  void TangoConnect();

  TangoConfig tango_config_;
  ros::NodeHandle node_handle_;
  PublisherConfiguration publisher_config_;
  bool pose_lock_ = false;
  bool point_cloud_lock_ = false;
  bool image_lock_ = false;
  bool new_pose_available_ = false;
  bool new_point_cloud_available_ = false;
  bool new_image_available_ = false;

  tf::TransformBroadcaster tf_broadcaster_;
  geometry_msgs::TransformStamped start_of_service_T_device_;
  geometry_msgs::TransformStamped start_of_service_T_camera_depth_;
  geometry_msgs::TransformStamped start_of_service_T_camera_;

  ros::Publisher point_cloud_publisher_;
  sensor_msgs::PointCloud2 point_cloud_;

  ros::Publisher image_publisher_;
  sensor_msgs::CompressedImage compressed_image_;
  cv::Mat image_;
};
}  // namespace tango_ros_node
#endif  // TANGO_ROS_NODE_H_
