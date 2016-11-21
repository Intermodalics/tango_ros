// Copyright 2016 Intermodalics All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef TANGO_ROS_NODE_H_
#define TANGO_ROS_NODE_H_
#include <jni.h>
#include <string>

#include <tango_client_api/tango_client_api.h>
#include <tango_support_api/tango_support_api.h>

#include <opencv2/core/core.hpp>

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

namespace tango_ros_node {
const int NUMBER_OF_FIELDS_IN_POINT_CLOUD = 4;
constexpr char CV_IMAGE_COMPRESSING_FORMAT[] = ".jpg";
constexpr char ROS_IMAGE_COMPRESSING_FORMAT[] = "jpeg";
const int IMAGE_COMPRESSING_QUALITY = 50;

// Camera bitfield values.
const uint32_t CAMERA_NONE = 0;
const uint32_t CAMERA_FISHEYE = (1 << 1);
const uint32_t CAMERA_COLOR = (1 << 2);

struct PublisherConfiguration {
  bool publish_device_pose = false;
  bool publish_point_cloud = false;
  uint32_t publish_camera = CAMERA_NONE;

  std::string point_cloud_topic = "tango/point_cloud";
  std::string fisheye_camera_topic = "tango/camera/fisheye/image_raw/compressed";
  std::string color_camera_topic = "tango/camera/color/image_raw/compressed";
};

class TangoRosNode {
 public:
  TangoRosNode(PublisherConfiguration publisher_config);
  ~TangoRosNode();
  bool isTangoVersionOk(JNIEnv* env, jobject activity);
  bool SetBinder(JNIEnv* env, jobject binder);
  bool OnTangoServiceConnected();
  void TangoDisconnect();
  void Publish();

  void OnPoseAvailable(const TangoPoseData* pose);
  void OnPointCloudAvailable(const TangoPointCloud* point_cloud);
  void OnFrameAvailable(TangoCameraId camera_id, const TangoImageBuffer* buffer);

 private:
  TangoErrorType TangoSetupConfig();
  TangoErrorType TangoConnect();

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

  double time_offset_ = 0.; // Offset between tango time and ros time in ms.

  tf::TransformBroadcaster tf_broadcaster_;
  geometry_msgs::TransformStamped start_of_service_T_device_;
  tf2_ros::StaticTransformBroadcaster tf_static_broadcaster_;
  geometry_msgs::TransformStamped device_T_camera_depth_;
  geometry_msgs::TransformStamped device_T_camera_fisheye_;
  geometry_msgs::TransformStamped device_T_camera_color_;

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
