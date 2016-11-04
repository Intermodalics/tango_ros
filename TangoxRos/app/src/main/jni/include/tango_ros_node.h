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
static int kNumberOfFieldInPointCloud = 4;
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

  std::string parent_frame_id = "start_of_service";
  std::string device_frame_id = "device";
  std::string point_cloud_frame_id = "camera_depth";
  std::string camera_fisheye_frame_id = "camera_fisheye";
  std::string camera_color_frame_id = "camera_color";
};

class TangoRosNode {
 public:
  TangoRosNode(bool publish_device_pose, bool publish_pointcloud, CameraType publish_camera);
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
  geometry_msgs::TransformStamped device_frame_;
  geometry_msgs::TransformStamped point_cloud_frame_;
  geometry_msgs::TransformStamped camera_frame_;

  ros::Publisher point_cloud_publisher_;
  sensor_msgs::PointCloud2 point_cloud_;

  ros::Publisher image_publisher_;
  sensor_msgs::CompressedImage compressed_image_;
  cv::Mat image_;
};
}  // namespace tango_ros_node
#endif  // TANGO_ROS_NODE_H_
