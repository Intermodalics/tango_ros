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
#ifndef TANGO_ROS_NODELET_H_
#define TANGO_ROS_NODELET_H_
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <time.h>

#include <tango_3d_reconstruction/tango_3d_reconstruction_api.h>
#include <tango_client_api/tango_client_api.h>
#include <tango_support/tango_support.h>

#include <opencv2/core/core.hpp>

#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <tango_ros_messages/GetMapName.h>
#include <tango_ros_messages/GetMapUuids.h>
#include <tango_ros_messages/LoadOccupancyGrid.h>
#include <tango_ros_messages/SaveMap.h>
#include <tango_ros_messages/TangoConnect.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

#include "tango_ros_native/PublisherConfig.h"

namespace tango_ros_native {
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

const std::string DATASET_DEFAULT_DIRECTORY = "/sdcard/tango_ros_streamer/datasets/";
const std::string OCCUPANCY_GRID_DEFAULT_DIRECTORY = "/sdcard/tango_ros_streamer/occupancy_grids/";
const int NUMBER_OF_STATIC_TRANSFORMS = 5;

const std::string TANGO_STATUS_TOPIC_NAME = "status";
const std::string POINT_CLOUD_TOPIC_NAME = "point_cloud";
const std::string LASER_SCAN_TOPIC_NAME = "laser_scan";
const std::string FISHEYE_IMAGE_TOPIC_NAME = "camera/fisheye_1/image_raw";
const std::string FISHEYE_RECTIFIED_IMAGE_TOPIC_NAME = "camera/fisheye_1/image_rect";
const std::string COLOR_IMAGE_TOPIC_NAME = "camera/color_1/image_raw";
const std::string COLOR_RECTIFIED_IMAGE_TOPIC_NAME = "camera/color_1/image_rect";
const std::string COLOR_MESH_TOPIC_NAME = "reconstruction/mesh";
const std::string OCCUPANCY_GRID_TOPIC_NAME = "reconstruction/occupancy_grid";
const std::string STATIC_OCCUPANCY_GRID_TOPIC_NAME = "static_occupancy_grid";
const std::string START_OF_SERVICE_T_DEVICE_TOPIC_NAME = "transform/start_of_service_T_device";
const std::string AREA_DESCRIPTION_T_START_OF_SERVICE_TOPIC_NAME = "transform/area_description_T_start_of_service";

const std::string CREATE_NEW_MAP_PARAM_NAME = "create_new_map";
const std::string LOCALIZATION_MODE_PARAM_NAME = "localization_mode";
const std::string LOCALIZATION_MAP_UUID_PARAM_NAME = "localization_map_uuid";
const std::string DATASET_DIRECTORY_PARAM_NAME = "dataset_directory";
const std::string DATASET_UUID_PARAM_NAME = "dataset_uuid";
const std::string ENABLE_DEPTH_PARAM_NAME = "enable_depth";
const std::string ENABLE_COLOR_CAMERA_PARAM_NAME = "enable_color_camera";
const std::string PUBLISH_POSE_ON_TF_PARAM_NAME = "publish_pose_on_tf";
const std::string ENABLE_3DR_MESH_PARAM_NAME = "enable_3dr_mesh";
const std::string ENABLE_3DR_OCCUPANCY_GRID_PARAM_NAME = "enable_3dr_occupancy_grid";
const std::string USE_TF_STATIC_PARAM_NAME = "use_tf_static";
const std::string START_OF_SERVICE_FRAME_ID_PARAM_NAME = "start_of_service_frame_id";
const std::string AREA_DESCRIPTION_FRAME_ID_PARAM_NAME = "area_description_frame_id";
const std::string OCCUPANCY_GRID_DIRECTORY_PARAM_NAME = "occupancy_grid_directory";

const std::string GET_MAP_NAME_SERVICE_NAME = "get_map_name";
const std::string GET_MAP_UUIDS_SERVICE_NAME = "get_map_uuids";
const std::string SAVE_MAP_SERVICE_NAME = "save_map";
const std::string LOAD_OCCUPANCY_GRID_SERVICE_NAME = "load_occupancy_grid";
const std::string CONNECT_SERVICE_NAME = "connect";

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

enum class TangoStatus {
  UNKNOWN = 0,
  SERVICE_NOT_CONNECTED,
  NO_FIRST_VALID_POSE,
  SERVICE_CONNECTED
};

struct PublishThread {
  std::thread publish_thread;
  std::mutex data_available_mutex;
  std::condition_variable data_available;
};

// Node collecting tango data and publishing it on ros topics.
class TangoRosNodelet : public ::nodelet::Nodelet {
 public:
  TangoRosNodelet();
  ~TangoRosNodelet();
  // Initialization function called when plugin is loaded.
  void onInit();
  // Gets the full list of map Uuids (Universally Unique IDentifier)
  // available on the device.
  // @param uuid_list to fill in
  // @return false if list could not be requested.
  bool GetAvailableMapUuidsList(std::vector<std::string>& uuid_list);
  // Gets the map name corresponding to a given map uuid.
  // @param map_uuid UUID to lookup
  // @param map_name to fill in
  // @return false if the uuid was not found
  bool GetMapNameFromUuid(const std::string& map_uuid, std::string& map_name);

  // Tries to get a first valid pose and sets the camera intrinsics.
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
  // Publishes the available data (device pose, point cloud, images, ...).
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
  // ROS service callback to get the user readable name from a given UUID.
  bool GetMapNameServiceCallback(const tango_ros_messages::GetMapName::Request& req,
                                 tango_ros_messages::GetMapName::Response& res);
  // ROS service callback to get a list of available ADF UUIDs and corresponding
  // user readable map names.
  bool GetMapUuidsServiceCallback(const tango_ros_messages::GetMapUuids::Request& req,
                                  tango_ros_messages::GetMapUuids::Response& res);
  // Function called when the LoadOccupancyGrid service is called.
  // Load the requested occupancy grid and publish it.
  bool LoadOccupancyGridServiceCallback(const tango_ros_messages::LoadOccupancyGrid::Request& req,
                             tango_ros_messages::LoadOccupancyGrid::Response& res);
  // Function called when the SaveMap service is called.
  // Save the current map (ADF) to disc with the given name.
  bool SaveMapServiceCallback(const tango_ros_messages::SaveMap::Request& req,
               tango_ros_messages::SaveMap::Response& res);
  // ROS service callback to connect or disconnect from Tango Service.
  bool TangoConnectServiceCallback(
          const tango_ros_messages::TangoConnect::Request& request,
          tango_ros_messages::TangoConnect::Response& response);

  TangoConfig tango_config_;
  ros::NodeHandle node_handle_;

  PublishThread device_pose_thread_;
  PublishThread point_cloud_thread_;
  PublishThread laser_scan_thread_;
  PublishThread fisheye_image_thread_;
  PublishThread color_image_thread_;
  PublishThread mesh_thread_;
  std::thread ros_spin_thread_;
  std::atomic_bool run_threads_;
  std::atomic_bool new_point_cloud_available_for_t3dr_;

  ros::Publisher tango_status_publisher_;
  TangoStatus tango_status_;

  bool tango_data_available_ = true;
  double time_offset_ = 0.; // Offset between tango time and ros time in s.
  bool publish_pose_on_tf_ = true;
  bool use_tf_static_ = true;
  bool enable_depth_ = true;
  bool enable_color_camera_ = true;
  bool enable_3dr_mesh_ = true;
  bool enable_3dr_occupancy_grid_ = true;
  bool fisheye_camera_available_ = true;
  std::string start_of_service_frame_id_;
  std::string area_description_frame_id_;

  tf::TransformBroadcaster tf_broadcaster_;
  ros::Publisher start_of_service_T_device_publisher_;
  ros::Publisher area_description_T_start_of_service_publisher_;
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

  ros::Publisher mesh_marker_publisher_;
  ros::Publisher occupancy_grid_publisher_;
  nav_msgs::OccupancyGrid occupancy_grid_;
  // Context for a 3D Reconstruction. Maintains the state of a single
  // mesh being reconstructed.
  Tango3DR_ReconstructionContext t3dr_context_;
  TangoSupport_PointCloudManager* point_cloud_manager_;
  Tango3DR_Pose last_camera_depth_pose_;
  TangoSupport_ImageBufferManager* image_buffer_manager_;
  Tango3DR_Pose last_camera_color_pose_;
  Tango3DR_CameraCalibration t3dr_color_camera_intrinsics_;
  double t3dr_resolution_;
  uint8_t t3dr_occupancy_grid_threshold_;

  ros::Publisher static_occupancy_grid_publisher_;

  ros::ServiceServer get_map_name_service_;
  ros::ServiceServer get_map_uuids_service_;
  ros::ServiceServer save_map_service_;
  ros::ServiceServer load_occupancy_grid_service_;
  ros::ServiceServer tango_connect_service_;
};
}  // namespace tango_ros_native
#endif  // TANGO_ROS_NODELET_H_
