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

#include <tango_3d_reconstruction/tango_3d_reconstruction_api.h>
#include <tango_client_api/tango_client_api.h>
#include <tango_support_api/tango_support_api.h>

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
#include <tango_ros_messages/SaveMap.h>
#include <tango_ros_messages/TangoConnect.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

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

const std::string TANGO_STATUS_TOPIC = "tango/status";
const std::string POINT_CLOUD_TOPIC_NAME = "tango/point_cloud";
const std::string LASER_SCAN_TOPIC_NAME = "tango/laser_scan";
const std::string FISHEYE_IMAGE_TOPIC_NAME = "tango/camera/fisheye_1/image_raw";
const std::string FISHEYE_RECTIFIED_IMAGE_TOPIC_NAME = "tango/camera/fisheye_1/image_rect";
const std::string COLOR_IMAGE_TOPIC_NAME = "tango/camera/color_1/image_raw";
const std::string COLOR_MESH_TOPIC_NAME = "tango/mesh";
const std::string COLOR_RECTIFIED_IMAGE_TOPIC_NAME = "tango/camera/color_1/image_rect";
const std::string CREATE_NEW_MAP_PARAM_NAME = "tango/create_new_map";
const std::string LOCALIZATION_MODE_PARAM_NAME = "tango/localization_mode";
const std::string LOCALIZATION_MAP_UUID_PARAM_NAME = "/tango/localization_map_uuid";
const std::string DATASET_PATH_PARAM_NAME = "tango/dataset_datasets_path";
const std::string DATASET_UUID_PARAM_NAME = "tango/dataset_uuid";
const std::string DATASETS_PATH = "/sdcard/tango_ros_streamer/datasets/";

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

// GridIndex makes indices into a value-struct instead of just an
// array.
struct GridIndex {
  Tango3DR_GridIndex indices;

  bool operator==(const GridIndex& other) const;
};

enum class TangoStatus {
  UNKNOWN = 0,
  SERVICE_NOT_BOUND,
  SERVICE_NOT_CONNECTED,
  SERVICE_CONNECTED,
  VERSION_NOT_SUPPORTED,
  SERVICE_RUNNING
};

// Node collecting tango data and publishing it on ros topics.
class TangoRosNode {
 public:
  TangoRosNode();
  ~TangoRosNode();
  // Gets the full list of map Uuids (Universally Unique IDentifier)
  // available on the device.
  // @return a list as a string: uuids are comma-separated.
  std::string GetAvailableMapUuidsList();
  // Gets the map name corresponding to a given map uuid.
  std::string GetMapNameFromUuid(const std::string& map_uuid);


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
  // Now that Tango is configured correctly, we also need to configure
  // 3D Reconstruction the way we want.
  Tango3DR_Status TangoSetup3DRConfig();
  // Connects to the tango service and to the necessary callbacks.
  // @return returns TANGO_SUCCESS if connecting to tango ended successfully
  // or if service was already connected.
  TangoErrorType TangoConnect();
  // Sets up config, connect to Tango service, starts publishing threads and
  // publishes first static transforms.
  TangoErrorType ConnectToTangoAndSetUpNode();
  // Helper function to update and publish the Tango status. Publishes always
  // for convenience, even if Tango status did not change.
  void UpdateAndPublishTangoStatus(const TangoStatus& status);
  // Publishes the necessary static transforms (device_T_camera_*).
  void PublishStaticTransforms();
  // Publishes the available data (device pose, point cloud, laser scan, images).
  void PublishDevicePose();
  void PublishPointCloud();
  void PublishLaserScan();
  void PublishFisheyeImage();
  void PublishColorImage();
  void PublishMesh();
  // Runs ros::spinOnce() in a loop to trigger subscribers callbacks (e.g. dynamic reconfigure).
  void RunRosSpin();
  // Function called when one of the dynamic reconfigure parameter is changed.
  // Updates the publisher configuration consequently.
  void DynamicReconfigureCallback(PublisherConfig &config, uint32_t level);
  // Function called when the SaveMap service is called.
  // Save the current map (ADF) to disc with the given name.
  bool SaveMap(tango_ros_messages::SaveMap::Request &req, tango_ros_messages::SaveMap::Response &res);
  bool TangoConnectServiceCallback(
          const tango_ros_messages::TangoConnect::Request &request,
          tango_ros_messages::TangoConnect::Response& response);

  TangoConfig tango_config_;
  ros::NodeHandle node_handle_;

  std::thread publish_device_pose_thread_;
  std::thread publish_pointcloud_thread_;
  std::thread publish_laserscan_thread_;
  std::thread publish_fisheye_image_thread_;
  std::thread publish_color_image_thread_;
  std::thread publish_mesh_thread_;
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
  double laser_scan_max_height_ = 1.0;
  double laser_scan_min_height_ = -1.0;

  ros::Publisher tango_status_publisher_;
  TangoStatus tango_status_;
  bool tango_data_available_ = true;

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


  ros::Publisher mesh_publisher_;
  visualization_msgs::MarkerArray mesh_marker_array_;
  std::mutex mesh_available_mutex_;
  std::condition_variable mesh_available_;
  std::atomic_bool new_point_cloud_available_for_t3dr_;
  // Context for a 3D Reconstruction. Maintains the state of a single
  // mesh being reconstructed.
  Tango3DR_Context t3dr_context_;
  TangoSupportPointCloudManager* point_cloud_manager_;
  // The most recent point cloud received.
  TangoPointCloud* last_point_cloud_;
  // The pose the the most recent point cloud received.
  // Only meaningful if point_cloud_available_for_td3r_ is true.
  Tango3DR_Pose t3dr_depth_pose_;
  TangoSupportImageBufferManager* image_buffer_manager_;
  TangoImageBuffer* last_image_buffer_;
  Tango3DR_Pose t3dr_image_pose_;
  // Constant camera intrinsics for the color camera.
  Tango3DR_CameraCalibration t3dr_intrinsics_;
  // Updated indices from the 3D Reconstruction library. The grids for
  // each of these needs to be re-extracted.
  std::vector<GridIndex> updated_indices_;
  bool use_floor_plan_ = false;

  ros::ServiceServer save_map_service_;
  ros::ServiceServer tango_connect_service_;
};
}  // namespace tango_ros_native
#endif  // TANGO_ROS_NODE_H_
