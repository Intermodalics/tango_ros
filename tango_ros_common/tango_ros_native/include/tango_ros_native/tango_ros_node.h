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
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <time.h>

#include <tango_client_api/tango_client_api.h>

#include <opencv2/core/core.hpp>

#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "tango_ros_native/PublisherConfig.h"

namespace tango_ros_native {
const std::string NODE_NAME = "tango";
const int NUMBER_OF_FIELDS_IN_POINT_CLOUD = 4;
// See laser scan message doc for definition of laser scan constants:
// http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
const float LASER_SCAN_ANGLE_MIN = 0.;
const float LASER_SCAN_ANGLE_MAX = 3.1415;
const float LASER_SCAN_ANGLE_INCREMENT = 3.1415 / 360;
const float LASER_SCAN_TIME_INCREMENT = 0.0;
const float LASER_SCAN_SCAN_TIME= 0.3333;
const float LASER_SCAN_RANGE_MIN = 0.15;
const float LASER_SCAN_RANGE_MAX = 4.0;
const std::string LASER_SCAN_FRAME_ID = "laser";

// Camera bitfield values.
const uint32_t CAMERA_NONE = 0;
const uint32_t CAMERA_FISHEYE = (1 << 1);
const uint32_t CAMERA_COLOR = (1 << 2);

// Localization mode values.
// See http://developers.google.com/tango/overview/area-learning to know more
// about Tango localization
enum LocalizationMode {
  // No map required. Internally runs VIO (Visual Inertial Odometry) from Tango.
  ODOMETRY = 1,
  // No map required. Internally runs COM (Concurrent Odometry and Mapping,
  // also mentioned as drift correction) from Tango.
  ONLINE_SLAM = 2,
  // Map required. Internally runs Tango localization on ADF (Area Description File).
  LOCALIZATION = 3
};

struct PublisherConfiguration {
  // True if pose needs to be published.
  std::atomic_bool publish_device_pose{false};
  // True if point cloud needs to be published.
  std::atomic_bool publish_point_cloud{false};
  // True if laser scan needs to be published.
  std::atomic_bool publish_laser_scan{false};
  // Flag corresponding to which cameras need to be published.
  std::atomic<uint32_t> publish_camera{CAMERA_NONE};

  // Topic name for the point cloud publisher.
  std::string point_cloud_topic = "tango/point_cloud";
  // Topic name for the laser scan publisher.
  std::string laser_scan_topic = "tango/laser_scan";
  // Topic name for the fisheye raw image publisher.
  std::string fisheye_image_topic = "tango/camera/fisheye_1/image_raw";
  // Topic name for the fisheye rectified image publisher.
  std::string fisheye_rectified_image_topic = "tango/camera/fisheye_1/image_rect";
  // Topic name for the color raw image publisher.
  std::string color_image_topic = "tango/camera/color_1/image_raw";
  // Topic name for the color rectified image publisher.
  std::string color_rectified_image_topic = "tango/camera/color_1/image_rect";
  // Param name for the drift correction parameter.
  std::string localization_mode_param = "tango/localization_mode";
};

// Node collecting tango data and publishing it on ros topics.
class TangoRosNode {
 public:
  TangoRosNode();
  TangoRosNode(const PublisherConfiguration& publisher_config);
  ~TangoRosNode();
  // Sets the tango config and connects to the tango service.
  // It also publishes the necessary static transforms (device_T_camera_*).
  // @return returns success if it ended successfully.
  TangoErrorType OnTangoServiceConnected();
  // Disconnects from the tango service.
  void TangoDisconnect();
  // Starts the threads that publish data.
  void StartPublishing();
  // Stops the threads that publish data.
  // Will not return until all the internal threads have exited.
  void StopPublishing();

  // Function called when a new device pose is available.
  void OnPoseAvailable(const TangoPoseData* pose);
  // Function called when a new point cloud is available.
  void OnPointCloudAvailable(const TangoPointCloud* point_cloud);
  // Function called when a new camera image is available.
  void OnFrameAvailable(TangoCameraId camera_id, const TangoImageBuffer* buffer);

 private:
  // Sets the tango config to be able to collect all necessary data from tango.
  // @return returns TANGO_SUCCESS if the config was set successfully.
  TangoErrorType TangoSetupConfig();
  // Connects to the tango service and to the necessary callbacks.
  // @return returns TANGO_SUCCESS if connecting to tango ended successfully.
  TangoErrorType TangoConnect();
  // Publishes the necessary static transforms (device_T_camera_*).
  void PublishStaticTransforms();
  // Publishes the available data (device pose, point cloud, laser scan, images).
  void PublishDevicePose();
  void PublishPointCloud();
  void PublishLaserScan();
  void PublishFisheyeImage();
  void PublishColorImage();
  // Runs ros::spinOnce() in a loop to trigger subscribers callbacks (e.g. dynamic reconfigure).
  void RunRosSpin();
  // Function called when one of the dynamic reconfigure parameter is changed.
  // Updates the publisher configuration consequently.
  void DynamicReconfigureCallback(PublisherConfig &config, uint32_t level);


  TangoConfig tango_config_;
  ros::NodeHandle node_handle_;

  PublisherConfiguration publisher_config_;
  std::thread publish_device_pose_thread_;
  std::thread publish_pointcloud_thread_;
  std::thread publish_laserscan_thread_;
  std::thread publish_fisheye_image_thread_;
  std::thread publish_color_image_thread_;
  std::thread ros_spin_thread_;
  std::atomic_bool run_threads_;

  std::mutex pose_available_mutex_;
  std::condition_variable pose_available_;
  std::mutex point_cloud_available_mutex_;
  std::condition_variable point_cloud_available_;
  std::mutex laser_scan_available_mutex_;
  std::condition_variable laser_scan_available_;
  std::mutex fisheye_image_available_mutex_;
  std::condition_variable fisheye_image_available_;
  std::mutex color_image_available_mutex_;
  std::condition_variable color_image_available_;

  double time_offset_ = 0.; // Offset between tango time and ros time in s.

  tf::TransformBroadcaster tf_broadcaster_;
  geometry_msgs::TransformStamped start_of_service_T_device_;
  geometry_msgs::TransformStamped area_description_T_start_of_service_;
  tf2_ros::StaticTransformBroadcaster tf_static_broadcaster_;
  geometry_msgs::TransformStamped device_T_camera_depth_;
  tf::StampedTransform camera_depth_T_laser_;
  geometry_msgs::TransformStamped device_T_camera_fisheye_;
  geometry_msgs::TransformStamped device_T_camera_color_;

  ros::Publisher point_cloud_publisher_;
  sensor_msgs::PointCloud2 point_cloud_;

  ros::Publisher laser_scan_publisher_;
  sensor_msgs::LaserScan laser_scan_;
  // TODO: make these ros params.
  double laser_scan_min_height_ = -1.0; // meter
  double laser_scan_max_height_ = 1.0; // meter

  std::shared_ptr<image_transport::ImageTransport> image_transport_;

  image_transport::CameraPublisher fisheye_camera_publisher_;
  std_msgs::Header fisheye_image_header_;
  sensor_msgs::CameraInfo fisheye_camera_info_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> fisheye_camera_info_manager_;
  image_transport::Publisher fisheye_rectified_image_publisher_;
  cv::Mat fisheye_image_;
  cv::Mat cv_warp_map_x_;
  cv::Mat cv_warp_map_y_;
  cv::Mat fisheye_image_rect_;

  image_transport::CameraPublisher color_camera_publisher_;
  std_msgs::Header color_image_header_;
  sensor_msgs::CameraInfo color_camera_info_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> color_camera_info_manager_;
  image_transport::Publisher color_rectified_image_publisher_;
  cv::Mat color_image_;
  image_geometry::PinholeCameraModel color_camera_model_;
  cv::Mat color_image_rect_;
};
}  // namespace tango_ros_native
#endif  // TANGO_ROS_NODE_H_
