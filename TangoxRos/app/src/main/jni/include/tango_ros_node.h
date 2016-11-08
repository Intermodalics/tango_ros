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
  bool publish_fisheye_camera = false;
  bool publish_color_camera = false;

  std::string point_cloud_topic = "tango/point_cloud";
  std::string fisheye_camera_topic = "tango/fisheye_camera/image_raw/compressed";
  std::string color_camera_topic = "tango/color_camera/image_raw/compressed";

  std::string parent_frame_id = "start_of_service";
  std::string device_frame_id = "device";
  std::string point_cloud_frame_id = "camera_depth";
  std::string fisheye_camera_frame_id = "camera_fisheye";
  std::string color_camera_frame_id = "camera_color";
};

class TangoRosNode {
 public:
  TangoRosNode(PublisherConfiguration publisher_config);
  ~TangoRosNode();
  void CheckTangoVersion(JNIEnv* env, jobject activity);
  void OnTangoServiceConnected(JNIEnv* env, jobject binder);
  void TangoDisconnect();
  void Publish();

  void OnPoseAvailable(const TangoPoseData* pose);
  void OnPointCloudAvailable(const TangoPointCloud* point_cloud);
  void OnFrameAvailable(TangoCameraId camera_id, const TangoImageBuffer* buffer);

 private:
  bool TangoSetupConfig();
  void TangoConnect();

  TangoConfig tango_config_;
  ros::NodeHandle node_handle_;
  PublisherConfiguration publisher_config_;
  bool pose_lock_ = false;
  bool point_cloud_lock_ = false;
  bool fisheye_image_lock_ = false;
  bool color_image_lock_ = false;
  bool new_pose_available_ = false;
  bool new_point_cloud_available_ = false;
  bool new_fisheye_image_available_ = false;
  bool new_color_image_available_ = false;

  tf::TransformBroadcaster tf_broadcaster_;
  geometry_msgs::TransformStamped device_frame_;
  geometry_msgs::TransformStamped point_cloud_frame_;
  geometry_msgs::TransformStamped fisheye_camera_frame_;
  geometry_msgs::TransformStamped color_camera_frame_;

  ros::Publisher point_cloud_publisher_;
  sensor_msgs::PointCloud2 point_cloud_;

  ros::Publisher fisheye_image_publisher_;
  sensor_msgs::CompressedImage fisheye_compressed_image_;
  cv::Mat fisheye_image_;

  ros::Publisher color_image_publisher_;
  sensor_msgs::CompressedImage color_compressed_image_;
  cv::Mat color_image_;
};
}  // namespace tango_ros_node
#endif  // TANGO_ROS_NODE_H_
