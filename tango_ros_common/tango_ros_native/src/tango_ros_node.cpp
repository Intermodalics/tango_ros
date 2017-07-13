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
#include "tango_ros_native/occupancy_grid_file_io.h"
#include "tango_ros_native/tango_3d_reconstruction_helper.h"
#include "tango_ros_native/tango_ros_conversions_helper.h"
#include "tango_ros_native/tango_ros_node.h"

#include <cmath>
#include <ctime>
#include <iostream>
#include <vector>
#include <sstream>

#include <glog/logging.h>

#include <dynamic_reconfigure/config_tools.h>
#include <dynamic_reconfigure/server.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int8.h>

PLUGINLIB_EXPORT_CLASS(tango_ros_native::TangoRosNode, nodelet::Nodelet)

namespace {
std::vector<std::string> SplitCommaSeparatedString(const std::string& comma_separated_string) {
  std::vector<std::string> output;
  std::stringstream ss(comma_separated_string);

  std::string string_element;
  while (std::getline(ss, string_element, ',')) {
    output.push_back(string_element);
  }
  return output;
}
// This function routes onPoseAvailable callback to the application object for
// handling.
// @param context, context will be a pointer to a TangoRosNode
//        instance on which to call the callback.
// @param pose, pose data to route to onPoseAvailable function.
void OnPoseAvailableRouter(void* context, const TangoPoseData* pose) {
  tango_ros_native::TangoRosNode* app =
      static_cast<tango_ros_native::TangoRosNode*>(context);
  app->OnPoseAvailable(pose);
}
// This function routes onPointCloudAvailable callback to the application
// object for handling.
// @param context, context will be a pointer to a TangoRosNode
//        instance on which to call the callback.
// @param point_cloud, point cloud data to route to OnPointCloudAvailable
//        function.
void OnPointCloudAvailableRouter(void* context,
                                 const TangoPointCloud* point_cloud) {
  tango_ros_native::TangoRosNode* app =
      static_cast<tango_ros_native::TangoRosNode*>(context);
  app->OnPointCloudAvailable(point_cloud);
}
// This function routes OnFrameAvailable callback to the application
// object for handling.
// @param context, context will be a pointer to a TangoRosNode
//        instance on which to call the callback.
// @param camera_id, the ID of the camera. Only TANGO_CAMERA_COLOR and
//        TANGO_CAMERA_FISHEYE are supported.
// @param buffer, image data to route to OnFrameAvailable function.
void OnFrameAvailableRouter(void* context, TangoCameraId camera_id,
                            const TangoImageBuffer* buffer) {
  tango_ros_native::TangoRosNode* app =
      static_cast<tango_ros_native::TangoRosNode*>(context);
  app->OnFrameAvailable(camera_id, buffer);
}

// Compute fisheye distorted coordinates from undistorted coordinates.
// The distortion model used by the Tango fisheye camera is called FOV and is
// described in 'Straight lines have to be straight' by Frederic Devernay and
// Olivier Faugeras. See https://hal.inria.fr/inria-00267247/document.
void ApplyFovModel(
    double xu, double yu, double w, double w_inverse, double two_tan_w_div_two,
    double* xd, double* yd) {
  double ru = sqrt(xu * xu + yu * yu);
  constexpr double epsilon = 1e-7;
  if (w < epsilon || ru < epsilon) {
    *xd = xu;
    *yd = yu ;
  } else {
    double rd_div_ru = std::atan(ru * two_tan_w_div_two) * w_inverse / ru;
    *xd = xu * rd_div_ru;
    *yd = yu * rd_div_ru;
  }
}
// Compute the warp maps to undistort the Tango fisheye image using the FOV
// model. See OpenCV documentation for more information on warp maps:
// http://docs.opencv.org/2.4/modules/imgproc/doc/geometric_transformations.html
// @param fisheye_camera_info the fisheye camera intrinsics.
// @param cv_warp_map_x the output map for the x direction.
// @param cv_warp_map_y the output map for the y direction.
void ComputeWarpMapsToRectifyFisheyeImage(
    const sensor_msgs::CameraInfo& fisheye_camera_info,
    cv::Mat* cv_warp_map_x, cv::Mat* cv_warp_map_y) {
  const double fx = fisheye_camera_info.K[0];
  const double fy = fisheye_camera_info.K[4];
  const double cx = fisheye_camera_info.K[2];
  const double cy = fisheye_camera_info.K[5];
  const double w = fisheye_camera_info.D[0];
  // Pre-computed variables for more efficiency.
  const double fy_inverse = 1.0 / fy;
  const double fx_inverse = 1.0 / fx;
  const double w_inverse = 1 / w;
  const double two_tan_w_div_two = 2.0 * std::tan(w * 0.5);
  // Compute warp maps in x and y directions.
  // OpenCV expects maps from dest to src, i.e. from undistorted to distorted
  // pixel coordinates.
  for(int iu = 0; iu < fisheye_camera_info.height; ++iu) {
    for (int ju = 0; ju < fisheye_camera_info.width; ++ju) {
      double xu = (ju - cx) * fx_inverse;
      double yu = (iu - cy) * fy_inverse;
      double xd, yd;
      ApplyFovModel(xu, yu, w, w_inverse, two_tan_w_div_two, &xd, &yd);
      double jd = cx + xd * fx;
      double id = cy + yd * fy;
      cv_warp_map_x->at<float>(iu, ju) = jd;
      cv_warp_map_y->at<float>(iu, ju) = id;
    }
  }
}
// Returns a string corresponding to current date and time.
// The format is as follow: year-month-day_hour-min-sec.
std::string GetCurrentDateAndTime() {
  std::time_t currentTime;
  struct tm* currentDateTime;
  std::time(&currentTime);
  currentDateTime = std::localtime(&currentTime);
  int day = currentDateTime->tm_mday;
  int month = currentDateTime->tm_mon + 1;
  int year = currentDateTime->tm_year + 1900;
  int hour = currentDateTime->tm_hour;
  int min = currentDateTime->tm_min;
  int sec = currentDateTime->tm_sec;
  std::ostringstream oss;
  oss << year << "-" << month << "-" << day << "_" << hour << "-" << min << "-" << sec;
  return oss.str();
}
// Returns device boottime in second.
double GetBootTimeInSecond() {
  struct timespec res_boot;
  clock_gettime(CLOCK_BOOTTIME, &res_boot);
  return res_boot.tv_sec + (double) res_boot.tv_nsec / 1e9;
}
// Save an Area Description File (ADF) and set its name.
// @param map_name Name of the ADF
// @param[out] map_uuid Uuid of the ADF.
// @param[out] message Contains an error message in case of failure.
// Returns true if the ADF was successfully saved and named, false otherwise.
bool SaveTangoAreaDescription(const std::string& map_name,
                              std::string& map_uuid, std::string& message) {
  TangoErrorType result;
  TangoUUID map_tango_uuid;
  result = TangoService_saveAreaDescription(&map_tango_uuid);
  if (result != TANGO_SUCCESS) {
    LOG(ERROR) << "Error while saving area description, error: " << result;
    message =  "Could not save the map. "
        "Did you allow the app to use area learning?";
    return false;
  }
  TangoAreaDescriptionMetadata metadata;
  result = TangoService_getAreaDescriptionMetadata(map_tango_uuid, &metadata);
  if (result != TANGO_SUCCESS) {
    LOG(ERROR) << "Error while trying to access area description metadata, error: " << result;
    message =  "Could not access map metadata";
    return false;
  }
  result = TangoAreaDescriptionMetadata_set(metadata, "name", map_name.capacity(), map_name.c_str());
  if (result != TANGO_SUCCESS) {
    LOG(ERROR) << "Error while trying to change area description metadata, error: " << result;
    message =  "Could not set the name of the map";
    return false;
  }
  result = TangoService_saveAreaDescriptionMetadata(map_tango_uuid, metadata);
  if (result != TANGO_SUCCESS) {
    LOG(ERROR) << "Error while saving new area description metadata, error: " << result;
    message =  "Could not save map metadata";
    return false;
  }
  result = TangoAreaDescriptionMetadata_free(metadata);
  if (result != TANGO_SUCCESS) {
    LOG(ERROR) << "Error while trying to free area description metadata, error: " << result;
    message =  "Could not free map metadata";
    return false;
  }
  map_uuid = static_cast<std::string>(map_tango_uuid);
  return true;
}
// Get the uuid of the ADF currently used by Tango.
// @param tango_config Current configuration of Tango.
// @param[out] adf_uuid Uuid of the current ADF, empty if Tango
// is not using an ADF for localization.
bool GetCurrentADFUuid(const TangoConfig& tango_config, std::string& adf_uuid) {
  char current_adf_uuid[TANGO_UUID_LEN];
  const char* config_load_area_description_UUID = "config_load_area_description_UUID";
  TangoErrorType result = TangoConfig_getString(
      tango_config, config_load_area_description_UUID, current_adf_uuid,
      TANGO_UUID_LEN);
  if (result != TANGO_SUCCESS) {
    LOG(ERROR) << "TangoConfig_getString "
        << config_load_area_description_UUID << " error: " << result;
    return false;
  }
  adf_uuid = std::string(current_adf_uuid);
  return true;
}

template<typename T>
void SetDefaultValueIfParamDoesNotExist(
    const ros::NodeHandle& node_handle, const std::string& param_name,
    T default_value) {
  if (!node_handle.hasParam(param_name)) {
    node_handle.setParam(param_name, default_value);
  }
}
template<typename T>
void GetParamValueAndSetDefaultValueIfParamDoesNotExist(
    const ros::NodeHandle& node_handle, const std::string& param_name,
    T default_value, T& param_value) {
  if (node_handle.hasParam(param_name)) {
    node_handle.getParam(param_name, param_value);
  } else {
    param_value = default_value;
    node_handle.setParam(param_name, default_value);
  }
}
}  // namespace

namespace tango_ros_native {
TangoRosNode::TangoRosNode() : run_threads_(false), tango_config_(nullptr),
    t3dr_context_(nullptr), point_cloud_manager_(nullptr),
    image_buffer_manager_(nullptr), new_point_cloud_available_for_t3dr_(false) {}

void TangoRosNode::onInit() {
  node_handle_ = getMTPrivateNodeHandle();
  const  uint32_t queue_size = 1;
  const bool latching = true;
  tango_status_publisher_ =
      node_handle_.advertise<std_msgs::Int8>(TANGO_STATUS_TOPIC_NAME,
                                             queue_size, latching);
  start_of_service_T_device_publisher_ =
      node_handle_.advertise<geometry_msgs::TransformStamped>(
          START_OF_SERVICE_T_DEVICE_TOPIC_NAME, queue_size, latching);
  area_description_T_start_of_service_publisher_ =
      node_handle_.advertise<geometry_msgs::TransformStamped>(
          AREA_DESCRIPTION_T_START_OF_SERVICE_TOPIC_NAME, queue_size, latching);

  image_transport_.reset(new image_transport::ImageTransport(node_handle_));
  try {
    fisheye_camera_publisher_ =
        image_transport_->advertiseCamera(FISHEYE_IMAGE_TOPIC_NAME,
                                          queue_size, latching);
    fisheye_rectified_image_publisher_ =
        image_transport_->advertise(FISHEYE_RECTIFIED_IMAGE_TOPIC_NAME,
                                   queue_size, latching);
  } catch (const image_transport::Exception& e) {
    LOG(ERROR) << "Error while creating fisheye image transport publishers" << e.what();
  }
  static_occupancy_grid_publisher_ = node_handle_.advertise<nav_msgs::OccupancyGrid>(
      STATIC_OCCUPANCY_GRID_TOPIC_NAME, queue_size, latching);

  get_map_name_service_ = node_handle_.advertiseService<tango_ros_messages::GetMapName::Request,
      tango_ros_messages::GetMapName::Response>(GET_MAP_NAME_SERVICE_NAME,
                                             boost::bind(&TangoRosNode::GetMapNameServiceCallback, this, _1, _2));

  get_map_uuids_service_ = node_handle_.advertiseService<tango_ros_messages::GetMapUuids::Request,
      tango_ros_messages::GetMapUuids::Response>(GET_MAP_UUIDS_SERVICE_NAME,
                                             boost::bind(&TangoRosNode::GetMapUuidsServiceCallback, this, _1, _2));

  save_map_service_ = node_handle_.advertiseService<tango_ros_messages::SaveMap::Request,
      tango_ros_messages::SaveMap::Response>(SAVE_MAP_SERVICE_NAME,
                                             boost::bind(&TangoRosNode::SaveMapServiceCallback, this, _1, _2));
  load_occupancy_grid_service_ = node_handle_.advertiseService<tango_ros_messages::LoadOccupancyGrid::Request,
        tango_ros_messages::LoadOccupancyGrid::Response>(LOAD_OCCUPANCY_GRID_SERVICE_NAME,
                                               boost::bind(&TangoRosNode::LoadOccupancyGridServiceCallback, this, _1, _2));

  tango_connect_service_ = node_handle_.advertiseService<tango_ros_messages::TangoConnect::Request,
          tango_ros_messages::TangoConnect::Response>(
              CONNECT_SERVICE_NAME, boost::bind(
                  &TangoRosNode::TangoConnectServiceCallback, this, _1, _2));

  tango_status_ = TangoStatus::UNKNOWN;

  SetDefaultValueIfParamDoesNotExist(
      node_handle_, CREATE_NEW_MAP_PARAM_NAME, false);
  SetDefaultValueIfParamDoesNotExist(
      node_handle_, LOCALIZATION_MODE_PARAM_NAME, 2);
  SetDefaultValueIfParamDoesNotExist(
      node_handle_, LOCALIZATION_MAP_UUID_PARAM_NAME, "");
  SetDefaultValueIfParamDoesNotExist(
       node_handle_, OCCUPANCY_GRID_DIRECTORY_PARAM_NAME,
       OCCUPANCY_GRID_DEFAULT_DIRECTORY);
  SetDefaultValueIfParamDoesNotExist(
      node_handle_, DATASET_DIRECTORY_PARAM_NAME, DATASET_DEFAULT_DIRECTORY);
  SetDefaultValueIfParamDoesNotExist(
      node_handle_, DATASET_UUID_PARAM_NAME, "");
  SetDefaultValueIfParamDoesNotExist(node_handle_,
      tango_3d_reconstruction_helper::TANGO_3DR_RESOLUTION_PARAM_NAME,
      tango_3d_reconstruction_helper::TANGO_3DR_DEFAULT_RESOLUTION);
  SetDefaultValueIfParamDoesNotExist(node_handle_,
      tango_3d_reconstruction_helper::TANGO_3DR_USE_SPACE_CLEARING_PARAM_NAME,
      tango_3d_reconstruction_helper::TANGO_3DR_DEFAULT_USE_SPACE_CLEARING);
  SetDefaultValueIfParamDoesNotExist(node_handle_,
      tango_3d_reconstruction_helper::TANGO_3DR_MIN_NUM_VERTICES_PARAM_NAME,
      tango_3d_reconstruction_helper::TANGO_3DR_DEFAULT_MIN_NUM_VERTICES);
  SetDefaultValueIfParamDoesNotExist(node_handle_,
      tango_3d_reconstruction_helper::TANGO_3DR_UPDATE_METHOD_PARAM_NAME,
      tango_3d_reconstruction_helper::TANGO_3DR_DEFAULT_UPDATE_METHOD);
  SetDefaultValueIfParamDoesNotExist(node_handle_,
      tango_3d_reconstruction_helper::TANGO_3DR_MAX_VOXEL_WEIGHT_PARAM_NAME,
      tango_3d_reconstruction_helper::TANGO_3DR_DEFAULT_MAX_VOXEL_WEIGHT);
  SetDefaultValueIfParamDoesNotExist(node_handle_,
      tango_3d_reconstruction_helper::TANGO_3DR_FLOORPLAN_MAX_ERROR_PARAM_NAME,
      tango_3d_reconstruction_helper::TANGO_3DR_DEFAULT_FLOORPLAN_MAX_ERROR);
  SetDefaultValueIfParamDoesNotExist(
      node_handle_, USE_TF_STATIC_PARAM_NAME, true);
  GetParamValueAndSetDefaultValueIfParamDoesNotExist(
      node_handle_, PUBLISH_POSE_ON_TF_PARAM_NAME, true, publish_pose_on_tf_);
  GetParamValueAndSetDefaultValueIfParamDoesNotExist(
      node_handle_, ENABLE_DEPTH_PARAM_NAME, true, enable_depth_);
  GetParamValueAndSetDefaultValueIfParamDoesNotExist(
      node_handle_, ENABLE_COLOR_CAMERA_PARAM_NAME, true, enable_color_camera_);
  GetParamValueAndSetDefaultValueIfParamDoesNotExist(
      node_handle_, ENABLE_3DR_MESH_PARAM_NAME, true, enable_3dr_mesh_);
  GetParamValueAndSetDefaultValueIfParamDoesNotExist(
      node_handle_, ENABLE_3DR_OCCUPANCY_GRID_PARAM_NAME, true, enable_3dr_occupancy_grid_);
  int t3dr_occupancy_grid_threshold =
      tango_3d_reconstruction_helper::TANGO_3DR_OCCUPANCY_GRID_DEFAULT_THRESHOLD;
  GetParamValueAndSetDefaultValueIfParamDoesNotExist(node_handle_,
      tango_3d_reconstruction_helper::TANGO_3DR_OCCUPANCY_GRID_THRESHOLD_PARAM_NAME,
      static_cast<int>(
          tango_3d_reconstruction_helper::TANGO_3DR_OCCUPANCY_GRID_DEFAULT_THRESHOLD),
      t3dr_occupancy_grid_threshold);
  t3dr_occupancy_grid_threshold_ = t3dr_occupancy_grid_threshold;
  GetParamValueAndSetDefaultValueIfParamDoesNotExist(
      node_handle_, START_OF_SERVICE_FRAME_ID_PARAM_NAME,
      tango_ros_conversions_helper::toFrameId(TANGO_COORDINATE_FRAME_START_OF_SERVICE),
      start_of_service_frame_id_);
  GetParamValueAndSetDefaultValueIfParamDoesNotExist(
      node_handle_, AREA_DESCRIPTION_FRAME_ID_PARAM_NAME,
      tango_ros_conversions_helper::toFrameId(TANGO_COORDINATE_FRAME_AREA_DESCRIPTION),
      area_description_frame_id_);
}

TangoRosNode::~TangoRosNode() {
  TangoDisconnect();
}

TangoErrorType TangoRosNode::OnTangoServiceConnected() {
  const  uint32_t queue_size = 1;
  const bool latching = true;
  // Advertise topics that need depth camera and/or color camera only if cameras are enable.
  if (enable_depth_) {
    point_cloud_publisher_ =
        node_handle_.advertise<sensor_msgs::PointCloud2>(
            POINT_CLOUD_TOPIC_NAME, queue_size, latching);
    laser_scan_publisher_ =
        node_handle_.advertise<sensor_msgs::LaserScan>(
            LASER_SCAN_TOPIC_NAME, queue_size, latching);
  } else {
    point_cloud_publisher_.shutdown();
    laser_scan_publisher_.shutdown();
  }
  if (enable_color_camera_) {
    try {
      color_camera_publisher_ =
          image_transport_->advertiseCamera(COLOR_IMAGE_TOPIC_NAME,
                                            queue_size, latching);
      color_rectified_image_publisher_ =
          image_transport_->advertise(COLOR_RECTIFIED_IMAGE_TOPIC_NAME,
                                      queue_size, latching);
      } catch (const image_transport::Exception& e) {
        LOG(ERROR) << "Error while creating color image transport publishers" << e.what();
      }
  } else {
    color_camera_publisher_.shutdown();
    color_rectified_image_publisher_.shutdown();
  }
  node_handle_.param<bool>(ENABLE_3DR_MESH_PARAM_NAME, enable_3dr_mesh_, true);
  if (enable_depth_ && enable_color_camera_ && enable_3dr_mesh_) {
    mesh_marker_publisher_ =
        node_handle_.advertise<visualization_msgs::MarkerArray>(
            COLOR_MESH_TOPIC_NAME, queue_size, latching);
  } else {
    mesh_marker_publisher_.shutdown();
  }
  node_handle_.param<bool>(ENABLE_3DR_OCCUPANCY_GRID_PARAM_NAME, enable_3dr_occupancy_grid_, true);
  if (enable_depth_ && enable_color_camera_ && enable_3dr_occupancy_grid_){
    occupancy_grid_publisher_ = node_handle_.advertise<nav_msgs::OccupancyGrid>(
        OCCUPANCY_GRID_TOPIC_NAME, queue_size, latching);
  } else {
    occupancy_grid_publisher_.shutdown();
  }

  TangoCoordinateFramePair pair;
  pair.base = TANGO_COORDINATE_FRAME_START_OF_SERVICE;
  pair.target = TANGO_COORDINATE_FRAME_DEVICE;
  TangoPoseData pose;
  time_t current_time = time(NULL);
  time_t end = current_time + 10;
  while (current_time < end) {
    TangoService_getPoseAtTime(0.0, pair, &pose);
    if (pose.status_code == TANGO_POSE_VALID) {
      break;
    }
    sleep(1);
    current_time = time(NULL);
  }
  if (pose.status_code != TANGO_POSE_VALID) {
    LOG(ERROR) << "Error, could not get a first valid pose.";
    return TANGO_INVALID;
  }
  time_offset_ = ros::Time::now().toSec() - GetBootTimeInSecond();

  TangoCameraIntrinsics tango_camera_intrinsics;
  TangoService_getCameraIntrinsics(TANGO_CAMERA_FISHEYE, &tango_camera_intrinsics);
  tango_ros_conversions_helper::toCameraInfo(tango_camera_intrinsics, &fisheye_camera_info_);
  fisheye_camera_info_manager_.reset(new camera_info_manager::CameraInfoManager(node_handle_));
  fisheye_camera_info_manager_->setCameraName("fisheye_1");
  // Cache warp maps for more efficiency.
  cv_warp_map_x_.create(fisheye_camera_info_.height, fisheye_camera_info_.width, CV_32FC1);
  cv_warp_map_y_.create(fisheye_camera_info_.height, fisheye_camera_info_.width, CV_32FC1);
  ComputeWarpMapsToRectifyFisheyeImage(fisheye_camera_info_, &cv_warp_map_x_, &cv_warp_map_y_);
  fisheye_image_rect_.create(fisheye_camera_info_.height, fisheye_camera_info_.width, CV_8UC1);

  TangoService_getCameraIntrinsics(TANGO_CAMERA_COLOR, &tango_camera_intrinsics);
  tango_ros_conversions_helper::toCameraInfo(tango_camera_intrinsics, &color_camera_info_);
  color_camera_info_manager_.reset(new camera_info_manager::CameraInfoManager(node_handle_));
  color_camera_info_manager_->setCameraName("color_1");
  // Cache camera model for more efficiency.
  color_camera_model_.fromCameraInfo(color_camera_info_);

  if (enable_3dr_mesh_ || enable_3dr_occupancy_grid_) {
    tango_ros_conversions_helper::toTango3DR_CameraCalibration(
        tango_camera_intrinsics, &t3dr_color_camera_intrinsics_);
    tango_3d_reconstruction_helper::TangoSetup3DRConfig(
        node_handle_, &t3dr_resolution_, &t3dr_context_, &t3dr_color_camera_intrinsics_);
  }
  return TANGO_SUCCESS;
}

TangoErrorType TangoRosNode::TangoSetupConfig() {
  const char* function_name = "TangoRosNode::TangoSetupConfig()";

  tango_config_ = TangoService_getConfig(TANGO_CONFIG_DEFAULT);
  if (tango_config_ == nullptr) {
    LOG(ERROR) << function_name << ", TangoService_getConfig error.";
    return TANGO_ERROR;
  }

  TangoErrorType result;
  const char* config_enable_motion_tracking = "config_enable_motion_tracking";
  result = TangoConfig_setBool(tango_config_, config_enable_motion_tracking, true);
  if(result != TANGO_SUCCESS) {
    LOG(ERROR) << function_name << ", TangoConfig_setBool "
        << config_enable_motion_tracking << " error: " << result;
    return result;
  }
  bool create_new_map;
  node_handle_.param(CREATE_NEW_MAP_PARAM_NAME, create_new_map, false);
  const char* config_enable_learning_mode = "config_enable_learning_mode";
  result = TangoConfig_setBool(tango_config_, config_enable_learning_mode, create_new_map);
  if (result != TANGO_SUCCESS) {
    LOG(ERROR) << function_name << ", TangoConfig_setBool "
        << config_enable_learning_mode << " error: " << result;
    return result;
  }
  if (!create_new_map) {
    bool enable_drift_correction = false;
    int localization_mode;
    node_handle_.param(LOCALIZATION_MODE_PARAM_NAME, localization_mode,
                       (int)LocalizationMode::ONLINE_SLAM);
    if (localization_mode == LocalizationMode::ONLINE_SLAM) {
      enable_drift_correction = true;
    }
    const char* config_enable_drift_correction = "config_enable_drift_correction";
    result = TangoConfig_setBool(tango_config_, config_enable_drift_correction, enable_drift_correction);
    if (result != TANGO_SUCCESS) {
      LOG(ERROR) << function_name << ", TangoConfig_setBool "
          << config_enable_drift_correction << " error: " << result;
      return result;
    }
    if (localization_mode == LocalizationMode::LOCALIZATION) {
      std::string map_uuid_to_load = "";
      node_handle_.param<std::string>(LOCALIZATION_MAP_UUID_PARAM_NAME, map_uuid_to_load, "");
      const char* config_load_area_description_UUID = "config_load_area_description_UUID";
      result = TangoConfig_setString(tango_config_, config_load_area_description_UUID, map_uuid_to_load.c_str());
      if (result != TANGO_SUCCESS) {
        LOG(ERROR) << function_name << ", TangoConfig_setString "
            << config_load_area_description_UUID << " error: " << result;
        return result;
      }
    }
  }
  const char* config_enable_auto_recovery = "config_enable_auto_recovery";
  result = TangoConfig_setBool(tango_config_, config_enable_auto_recovery, true);
  if (result != TANGO_SUCCESS) {
    LOG(ERROR) << function_name << ", TangoConfig_setBool "
        << config_enable_auto_recovery << " error: " << result;
    return result;
  }
  const char* config_enable_depth = "config_enable_depth";
  node_handle_.param<bool>(ENABLE_DEPTH_PARAM_NAME, enable_depth_, true);
  result = TangoConfig_setBool(tango_config_, config_enable_depth, enable_depth_);
  if (result != TANGO_SUCCESS) {
    LOG(ERROR) << function_name << ", TangoConfig_setBool "
        << config_enable_depth << " error: " << result;
    return result;
  }
  const char* config_depth_mode = "config_depth_mode";
  result = TangoConfig_setInt32(tango_config_, config_depth_mode, TANGO_POINTCLOUD_XYZC);
  if (result != TANGO_SUCCESS) {
    LOG(ERROR) << function_name << ", TangoConfig_setInt "
        << config_depth_mode << " error: " << result;
    return result;
  }
  const char* config_enable_color_camera = "config_enable_color_camera";
  node_handle_.param<bool>(ENABLE_COLOR_CAMERA_PARAM_NAME, enable_color_camera_, true);
  result = TangoConfig_setBool(tango_config_, config_enable_color_camera, enable_color_camera_);
  if (result != TANGO_SUCCESS) {
    LOG(ERROR) << function_name << ", TangoConfig_setBool "
        << config_enable_color_camera << " error: " << result;
    return result;
  }
  std::string datasets_path;
  node_handle_.param(DATASET_DIRECTORY_PARAM_NAME, datasets_path, DATASET_DEFAULT_DIRECTORY);
  const char* config_datasets_path = "config_datasets_path";
  result = TangoConfig_setString(tango_config_, config_datasets_path, datasets_path.c_str());
  if (result != TANGO_SUCCESS) {
    LOG(ERROR) << function_name << ", TangoConfig_setString "
               << config_datasets_path << " error: " << result;
    return result;
  }
  std::string dataset_uuid;
  node_handle_.param(DATASET_UUID_PARAM_NAME, dataset_uuid, std::string(""));
  const char* config_experimental_load_dataset_UUID = "config_experimental_load_dataset_UUID";
  result = TangoConfig_setString(tango_config_, config_experimental_load_dataset_UUID, dataset_uuid.c_str());
  if (result != TANGO_SUCCESS) {
    LOG(ERROR) << function_name << ", TangoConfig_setString "
               << config_experimental_load_dataset_UUID << " error: " << result;
    return result;
  }
  if (point_cloud_manager_ == nullptr) {
    int32_t max_point_cloud_elements;
    const char* config_max_point_cloud_elements = "max_point_cloud_elements";
    result = TangoConfig_getInt32(tango_config_, config_max_point_cloud_elements,
                               &max_point_cloud_elements);
    if (result != TANGO_SUCCESS) {
      LOG(ERROR) << function_name << ", Failed to query maximum number of point cloud elements.";
      return result;
    }
    result = TangoSupport_createPointCloudManager(max_point_cloud_elements, &point_cloud_manager_);
    if (result != TANGO_SUCCESS) {
      LOG(ERROR) << function_name << ", Failed to create point cloud manager.";
      return result;
    }
  }
  return TANGO_SUCCESS;
}

TangoErrorType TangoRosNode::TangoConnect() {
  const char* function_name = "TangoRosNode::TangoConnect()";

  TangoCoordinateFramePair pairs[2] = {
         {TANGO_COORDINATE_FRAME_START_OF_SERVICE,
          TANGO_COORDINATE_FRAME_DEVICE},
         {TANGO_COORDINATE_FRAME_AREA_DESCRIPTION,
          TANGO_COORDINATE_FRAME_START_OF_SERVICE}};
  TangoErrorType result =
      TangoService_connectOnPoseAvailable(2, pairs, OnPoseAvailableRouter);
  if (result != TANGO_SUCCESS) {
    LOG(ERROR) << function_name
        << ", TangoService_connectOnPoseAvailable error: " << result;
    return result;
  }

  result = TangoService_connectOnPointCloudAvailable(OnPointCloudAvailableRouter);
  if (result != TANGO_SUCCESS) {
    LOG(ERROR) << function_name
        << ", TangoService_connectOnPointCloudAvailable error: " << result;
    return result;
  }

  result = TangoService_connectOnFrameAvailable(
      TANGO_CAMERA_FISHEYE, this, OnFrameAvailableRouter);
  if (result != TANGO_SUCCESS) {
    LOG(ERROR) << function_name
        << ", TangoService_connectOnFrameAvailable TANGO_CAMERA_FISHEYE error: " << result;
    return result;
  }

  result = TangoService_connectOnFrameAvailable(
      TANGO_CAMERA_COLOR, this, OnFrameAvailableRouter);
  if (result != TANGO_SUCCESS) {
    LOG(ERROR) << function_name
        << ", TangoService_connectOnFrameAvailable TANGO_CAMERA_COLOR error: " << result;
    return result;
  }

  result = TangoService_connect(this, tango_config_);
  if (result != TANGO_SUCCESS) {
    LOG(ERROR) << function_name << ", TangoService_connect error: " << result;
    return result;
  }
  return TANGO_SUCCESS;
}

void TangoRosNode::UpdateAndPublishTangoStatus(const TangoStatus& status) {
  tango_status_ = status;
  std_msgs::Int8 tango_status_msg;
  tango_status_msg.data = static_cast<int>(tango_status_);
  tango_status_publisher_.publish(tango_status_msg);
}

TangoErrorType TangoRosNode::ConnectToTangoAndSetUpNode() {
  if (tango_status_ == TangoStatus::SERVICE_CONNECTED) {
    // Connecting twice to Tango results in TANGO_ERROR. Early return to avoid
    // this.
    LOG(WARNING) << "Already connected to Tango service.";
    UpdateAndPublishTangoStatus(TangoStatus::SERVICE_CONNECTED);
    return TANGO_SUCCESS;
  }
  UpdateAndPublishTangoStatus(TangoStatus::UNKNOWN);

  TangoErrorType success;
  // Setup config.
  success = TangoSetupConfig();
  // Early return if config setup failed.
  if (success != TANGO_SUCCESS) {
    UpdateAndPublishTangoStatus(TangoStatus::SERVICE_NOT_CONNECTED);
    return success;
  }
  // Connect to Tango.
  success = TangoConnect();
  // Early return if Tango connect call failed.
  if (success != TANGO_SUCCESS) {
    UpdateAndPublishTangoStatus(TangoStatus::SERVICE_NOT_CONNECTED);
    return success;
  }
  // Publish static transforms.
  node_handle_.param(USE_TF_STATIC_PARAM_NAME, use_tf_static_, true);
  PublishStaticTransforms();
  success = OnTangoServiceConnected();
  if (success != TANGO_SUCCESS) {
    UpdateAndPublishTangoStatus(TangoStatus::NO_FIRST_VALID_POSE);
    return success;
  }
  area_description_T_start_of_service_.child_frame_id = "";
  tango_data_available_ = true;
  // Create publishing threads.
  StartPublishing();
  UpdateAndPublishTangoStatus(TangoStatus::SERVICE_CONNECTED);
  return success;
}

void TangoRosNode::TangoDisconnect() {
  StopPublishing();
  if (tango_config_ != nullptr) {
    TangoConfig_free(tango_config_);
    tango_config_ = nullptr;
  }
  if (point_cloud_manager_ != nullptr) {
    TangoSupport_freePointCloudManager(point_cloud_manager_);
    point_cloud_manager_ = nullptr;
  }
  if (image_buffer_manager_ != nullptr) {
    TangoSupport_freeImageBufferManager(image_buffer_manager_);
    image_buffer_manager_ = nullptr;
  }
  if (t3dr_context_ != nullptr) {
    Tango3DR_clear(t3dr_context_);
    Tango3DR_ReconstructionContext_destroy(t3dr_context_);
    t3dr_context_ = nullptr;
  }
  TangoService_disconnect();
  UpdateAndPublishTangoStatus(TangoStatus::SERVICE_NOT_CONNECTED);
}

void TangoRosNode::PublishStaticTransforms() {
  std::vector<geometry_msgs::TransformStamped> tf_transforms;
  tf_transforms.reserve(NUMBER_OF_STATIC_TRANSFORMS);
  TangoCoordinateFramePair pair;
  TangoPoseData pose;

  pair.base = TANGO_COORDINATE_FRAME_DEVICE;
  pair.target = TANGO_COORDINATE_FRAME_IMU;
  TangoService_getPoseAtTime(0.0, pair, &pose);
  geometry_msgs::TransformStamped device_T_imu;
  tango_ros_conversions_helper::toTransformStamped(
      pose, time_offset_,
      tango_ros_conversions_helper::toFrameId(TANGO_COORDINATE_FRAME_DEVICE),
      tango_ros_conversions_helper::toFrameId(TANGO_COORDINATE_FRAME_IMU),
      &device_T_imu);
  tf_transforms.push_back(device_T_imu);

  pair.base = TANGO_COORDINATE_FRAME_DEVICE;
  pair.target = TANGO_COORDINATE_FRAME_CAMERA_DEPTH;
  TangoService_getPoseAtTime(0.0, pair, &pose);
  tango_ros_conversions_helper::toTransformStamped(
      pose, time_offset_,
      tango_ros_conversions_helper::toFrameId(TANGO_COORDINATE_FRAME_DEVICE),
      tango_ros_conversions_helper::toFrameId(TANGO_COORDINATE_FRAME_CAMERA_DEPTH),
      &device_T_camera_depth_);
  tf_transforms.push_back(device_T_camera_depth_);

  // According to the ROS documentation, laser scan angles are measured around
  // the Z-axis in the laser scan frame. To follow this convention the laser
  // scan frame has to be rotated of 90 degrees around x axis with respect to
  // the Tango point cloud frame.
  camera_depth_T_laser_ = tf::StampedTransform(
      tf::Transform(tf::Quaternion(1 / sqrt(2), 0, 0, 1 / sqrt(2))), ros::Time::now(),
      tango_ros_conversions_helper::toFrameId(TANGO_COORDINATE_FRAME_CAMERA_DEPTH),
      LASER_SCAN_FRAME_ID);
  geometry_msgs::TransformStamped camera_depth_T_laser_message;
  tf::transformStampedTFToMsg(camera_depth_T_laser_, camera_depth_T_laser_message);
  tf_transforms.push_back(camera_depth_T_laser_message);

  pair.base = TANGO_COORDINATE_FRAME_DEVICE;
  pair.target = TANGO_COORDINATE_FRAME_CAMERA_FISHEYE;
  TangoService_getPoseAtTime(0.0, pair, &pose);
  tango_ros_conversions_helper::toTransformStamped(
      pose, time_offset_,
      tango_ros_conversions_helper::toFrameId(TANGO_COORDINATE_FRAME_DEVICE),
      tango_ros_conversions_helper::toFrameId(TANGO_COORDINATE_FRAME_CAMERA_FISHEYE),
      &device_T_camera_fisheye_);
  tf_transforms.push_back(device_T_camera_fisheye_);

  pair.base = TANGO_COORDINATE_FRAME_DEVICE;
  pair.target = TANGO_COORDINATE_FRAME_CAMERA_COLOR;
  TangoService_getPoseAtTime(0.0, pair, &pose);
  tango_ros_conversions_helper::toTransformStamped(
      pose, time_offset_,
      tango_ros_conversions_helper::toFrameId(TANGO_COORDINATE_FRAME_DEVICE),
      tango_ros_conversions_helper::toFrameId(TANGO_COORDINATE_FRAME_CAMERA_COLOR),
      &device_T_camera_color_);
  tf_transforms.push_back(device_T_camera_color_);

  if (use_tf_static_) {
    tf_static_broadcaster_.sendTransform(tf_transforms);
  } else {
    tf_broadcaster_.sendTransform(tf_transforms);
  }
}

void TangoRosNode::OnPoseAvailable(const TangoPoseData* pose) {
  if (publish_pose_on_tf_ ||
      start_of_service_T_device_publisher_.getNumSubscribers() > 0 ||
      area_description_T_start_of_service_publisher_.getNumSubscribers() > 0) {
    if (pose->frame.base == TANGO_COORDINATE_FRAME_START_OF_SERVICE &&
        pose->frame.target == TANGO_COORDINATE_FRAME_DEVICE) {
      if (pose->status_code == TANGO_POSE_VALID &&
          device_pose_thread_.data_available_mutex.try_lock()) {
        tango_ros_conversions_helper::toTransformStamped(
            *pose, time_offset_, start_of_service_frame_id_,
            tango_ros_conversions_helper::toFrameId(TANGO_COORDINATE_FRAME_DEVICE),
            &start_of_service_T_device_);
        device_pose_thread_.data_available_mutex.unlock();
        device_pose_thread_.data_available.notify_all();
      }
    } else if (pose->frame.base == TANGO_COORDINATE_FRAME_AREA_DESCRIPTION &&
        pose->frame.target == TANGO_COORDINATE_FRAME_START_OF_SERVICE) {
      if (pose->status_code == TANGO_POSE_VALID &&
          device_pose_thread_.data_available_mutex.try_lock()) {
        tango_ros_conversions_helper::toTransformStamped(
            *pose, time_offset_, area_description_frame_id_,
            start_of_service_frame_id_, &area_description_T_start_of_service_);
        device_pose_thread_.data_available_mutex.unlock();
        device_pose_thread_.data_available.notify_all();
      }
    }
  }
}

void TangoRosNode::OnPointCloudAvailable(const TangoPointCloud* point_cloud) {
  if (point_cloud->num_points > 0) {
    if (point_cloud_publisher_.getNumSubscribers() > 0 &&
        point_cloud_thread_.data_available_mutex.try_lock()) {
      tango_ros_conversions_helper::toPointCloud2(*point_cloud, time_offset_,
                                                  &point_cloud_);
      point_cloud_.header.frame_id =
          tango_ros_conversions_helper::toFrameId(TANGO_COORDINATE_FRAME_CAMERA_DEPTH);
      point_cloud_thread_.data_available_mutex.unlock();
      point_cloud_thread_.data_available.notify_all();
    }
    if (laser_scan_publisher_.getNumSubscribers() > 0 &&
        laser_scan_thread_.data_available_mutex.try_lock()) {
      laser_scan_.angle_min = LASER_SCAN_ANGLE_MIN;
      laser_scan_.angle_max = LASER_SCAN_ANGLE_MAX;
      laser_scan_.angle_increment = LASER_SCAN_ANGLE_INCREMENT;
      laser_scan_.time_increment = LASER_SCAN_TIME_INCREMENT;
      laser_scan_.scan_time = LASER_SCAN_SCAN_TIME;
      laser_scan_.range_min = LASER_SCAN_RANGE_MIN;
      laser_scan_.range_max = LASER_SCAN_RANGE_MAX;
      // Determine amount of rays to create.
      uint32_t ranges_size = std::ceil((laser_scan_.angle_max - laser_scan_.angle_min)
                                       / laser_scan_.angle_increment);
      // Laser scan rays with no obstacle data will evaluate to infinity.
      laser_scan_.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
      tango_ros_conversions_helper::toLaserScan(
          *point_cloud, time_offset_, laser_scan_min_height_,
                  laser_scan_max_height_, camera_depth_T_laser_, &laser_scan_);
      laser_scan_.header.frame_id = LASER_SCAN_FRAME_ID;
      laser_scan_thread_.data_available_mutex.unlock();
      laser_scan_thread_.data_available.notify_all();
    }

    if (enable_3dr_mesh_ || enable_3dr_occupancy_grid_) {
      TangoCoordinateFramePair pair;
      pair.base = TANGO_COORDINATE_FRAME_START_OF_SERVICE;
      pair.target = TANGO_COORDINATE_FRAME_CAMERA_DEPTH;
      TangoPoseData start_of_service_T_camera_depth;
      TangoService_getPoseAtTime(point_cloud->timestamp, pair, &start_of_service_T_camera_depth);
      if (start_of_service_T_camera_depth.status_code != TANGO_POSE_VALID) {
        LOG(WARNING) << "Could not find a valid pose at time "
            << point_cloud->timestamp << " for the depth camera.";
        return;
      }
      tango_ros_conversions_helper::toTango3DR_Pose(start_of_service_T_camera_depth,
                                            &last_camera_depth_pose_);
      TangoSupport_updatePointCloud(point_cloud_manager_, point_cloud);
      new_point_cloud_available_for_t3dr_ = true;
    }
  }
}

void TangoRosNode::OnFrameAvailable(TangoCameraId camera_id, const TangoImageBuffer* buffer) {
  if (fisheye_camera_publisher_.getNumSubscribers() > 0 &&
       camera_id == TangoCameraId::TANGO_CAMERA_FISHEYE &&
       fisheye_image_thread_.data_available_mutex.try_lock()) {
    fisheye_image_ = cv::Mat(buffer->height + buffer->height / 2, buffer->width,
                             CV_8UC1, buffer->data, buffer->stride); // No deep copy.
    fisheye_image_header_.stamp.fromSec(buffer->timestamp + time_offset_);
    fisheye_image_header_.seq = buffer->frame_number;
    fisheye_image_header_.frame_id =
        tango_ros_conversions_helper::toFrameId(TANGO_COORDINATE_FRAME_CAMERA_FISHEYE);
    fisheye_image_thread_.data_available_mutex.unlock();
    fisheye_image_thread_.data_available.notify_all();
  }
  if (color_camera_publisher_.getNumSubscribers() > 0 &&
       camera_id == TangoCameraId::TANGO_CAMERA_COLOR &&
       color_image_thread_.data_available_mutex.try_lock()) {
    color_image_ = cv::Mat(buffer->height + buffer->height / 2, buffer->width,
                           CV_8UC1, buffer->data, buffer->stride); // No deep copy.
    color_image_header_.stamp.fromSec(buffer->timestamp + time_offset_);
    color_image_header_.seq = buffer->frame_number;
    color_image_header_.frame_id =
        tango_ros_conversions_helper::toFrameId(TANGO_COORDINATE_FRAME_CAMERA_COLOR);
    color_image_thread_.data_available_mutex.unlock();
    color_image_thread_.data_available.notify_all();
  }

  if ((enable_3dr_mesh_ || enable_3dr_occupancy_grid_) &&
      camera_id == TangoCameraId::TANGO_CAMERA_COLOR &&
      new_point_cloud_available_for_t3dr_ &&
      mesh_thread_.data_available_mutex.try_lock()) {
    if (image_buffer_manager_ == nullptr) {
      TangoErrorType result = TangoSupport_createImageBufferManager(
          buffer->format, buffer->width, buffer->height, &image_buffer_manager_);
      if (result != TANGO_SUCCESS) {
        LOG(ERROR) << "Failed to create image buffer manager.";
        return;
      }
    }
    TangoCoordinateFramePair pair;
    pair.base = TANGO_COORDINATE_FRAME_START_OF_SERVICE;
    pair.target = TANGO_COORDINATE_FRAME_CAMERA_COLOR;
    TangoPoseData start_of_service_T_camera_color;
    TangoService_getPoseAtTime(buffer->timestamp, pair, &start_of_service_T_camera_color);
    if (start_of_service_T_camera_color.status_code != TANGO_POSE_VALID) {
      LOG(WARNING) << "Could not find a valid pose at time "
          << buffer->timestamp << " for the color camera.";
      return;
    }
    tango_ros_conversions_helper::toTango3DR_Pose(start_of_service_T_camera_color,
                                          &last_camera_color_pose_);
    TangoSupport_updateImageBuffer(image_buffer_manager_, buffer);
    new_point_cloud_available_for_t3dr_ = false;
    mesh_thread_.data_available_mutex.unlock();
    mesh_thread_.data_available.notify_all();
  }
}

void TangoRosNode::StartPublishing() {
  run_threads_ = true;
  device_pose_thread_.publish_thread =
      std::thread(&TangoRosNode::PublishDevicePose, this);
  point_cloud_thread_.publish_thread =
      std::thread(&TangoRosNode::PublishPointCloud, this);
  laser_scan_thread_.publish_thread =
      std::thread(&TangoRosNode::PublishLaserScan, this);
  fisheye_image_thread_.publish_thread =
      std::thread(&TangoRosNode::PublishFisheyeImage, this);
  color_image_thread_.publish_thread =
      std::thread(&TangoRosNode::PublishColorImage, this);
  mesh_thread_.publish_thread =
      std::thread(&TangoRosNode::PublishMesh, this);
  ros_spin_thread_ = std::thread(&TangoRosNode::RunRosSpin, this);
}

void TangoRosNode::StopPublishing() {
  if (run_threads_) {
    run_threads_ = false;
    if (device_pose_thread_.publish_thread.joinable()) {
      if (!tango_data_available_ || (!publish_pose_on_tf_ &&
          start_of_service_T_device_publisher_.getNumSubscribers() <= 0 &&
          area_description_T_start_of_service_publisher_.getNumSubscribers() <= 0)) {
        device_pose_thread_.data_available.notify_all();
      }
      device_pose_thread_.publish_thread.join();
    }
    if (point_cloud_thread_.publish_thread.joinable()) {
      if (!tango_data_available_ || !enable_depth_ ||
          point_cloud_publisher_.getNumSubscribers() <= 0) {
        point_cloud_thread_.data_available.notify_all();
      }
      point_cloud_thread_.publish_thread.join();
    }
    if (laser_scan_thread_.publish_thread.joinable()) {
      if (!tango_data_available_ || !enable_depth_ ||
          laser_scan_publisher_.getNumSubscribers() <= 0) {
        laser_scan_thread_.data_available.notify_all();
      }
      laser_scan_thread_.publish_thread.join();
    }
    if (fisheye_image_thread_.publish_thread.joinable()) {
      if (!tango_data_available_ ||
          fisheye_camera_publisher_.getNumSubscribers() <= 0) {
        fisheye_image_thread_.data_available.notify_all();
      }
      fisheye_image_thread_.publish_thread.join();
    }
    if (color_image_thread_.publish_thread.joinable()) {
      if (!tango_data_available_ || !enable_color_camera_ ||
          color_camera_publisher_.getNumSubscribers() <= 0) {
        color_image_thread_.data_available.notify_all();
      }
      color_image_thread_.publish_thread.join();
    }
    if (mesh_thread_.publish_thread.joinable()) {
      if (!tango_data_available_ || !enable_depth_ || !enable_color_camera_ ||
          !(enable_3dr_mesh_ && enable_3dr_occupancy_grid_)) {
        mesh_thread_.data_available.notify_all();
      }
      mesh_thread_.publish_thread.join();
    }
    ros_spin_thread_.join();
  }
}

void TangoRosNode::PublishDevicePose() {
  while(ros::ok()) {
    if (!run_threads_) {
      break;
    }
    {
      std::unique_lock<std::mutex> lock(device_pose_thread_.data_available_mutex);
      device_pose_thread_.data_available.wait(lock);
      if (!use_tf_static_) {
        PublishStaticTransforms();
      }
      if (publish_pose_on_tf_) {
        tf_broadcaster_.sendTransform(start_of_service_T_device_);
        if (area_description_T_start_of_service_.child_frame_id != "") {
          // This transform can be empty. Don't publish it in this case.
          tf_broadcaster_.sendTransform(area_description_T_start_of_service_);
        }
      }
      if (start_of_service_T_device_publisher_.getNumSubscribers() > 0) {
        start_of_service_T_device_publisher_.publish(start_of_service_T_device_);
      }
      if (area_description_T_start_of_service_publisher_.getNumSubscribers() > 0 &&
          area_description_T_start_of_service_.child_frame_id != "") {
        // This transform can be empty. Don't publish it in this case.
        area_description_T_start_of_service_publisher_.publish(area_description_T_start_of_service_);
      }
    }
  }
}

void TangoRosNode::PublishPointCloud() {
  while(ros::ok()) {
    if (!run_threads_) {
      break;
    }
    {
      std::unique_lock<std::mutex> lock(point_cloud_thread_.data_available_mutex);
      point_cloud_thread_.data_available.wait(lock);
      if (point_cloud_publisher_.getNumSubscribers() > 0) {
        point_cloud_publisher_.publish(point_cloud_);
      }
    }
  }
}

void TangoRosNode::PublishLaserScan() {
  while(ros::ok()) {
    if (!run_threads_) {
      break;
    }
    {
      std::unique_lock<std::mutex> lock(laser_scan_thread_.data_available_mutex);
      laser_scan_thread_.data_available.wait(lock);
      if (laser_scan_publisher_.getNumSubscribers() > 0) {
        laser_scan_publisher_.publish(laser_scan_);
      }
    }
  }
}

void TangoRosNode::PublishFisheyeImage() {
  while(ros::ok()) {
    if (!run_threads_) {
      break;
    }
    {
      std::unique_lock<std::mutex> lock(fisheye_image_thread_.data_available_mutex);
      fisheye_image_thread_.data_available.wait(lock);
      if (fisheye_camera_publisher_.getNumSubscribers() > 0 ||
          fisheye_rectified_image_publisher_.getNumSubscribers() > 0) {
        // The Tango image encoding is not supported by ROS.
        // We need to convert it to gray.
        cv::Mat fisheye_image_gray;
        cv::cvtColor(fisheye_image_, fisheye_image_gray, cv::COLOR_YUV420sp2GRAY);
        cv_bridge::CvImage cv_bridge_fisheye_image;
        cv_bridge_fisheye_image.header = fisheye_image_header_;
        cv_bridge_fisheye_image.encoding = sensor_msgs::image_encodings::MONO8;
        cv_bridge_fisheye_image.image = fisheye_image_gray;
        fisheye_camera_info_.header = fisheye_image_header_;
        fisheye_camera_info_manager_->setCameraInfo(fisheye_camera_info_);
        sensor_msgs::Image fisheye_image_msg;
        cv_bridge_fisheye_image.toImageMsg(fisheye_image_msg);
        fisheye_camera_publisher_.publish(fisheye_image_msg, fisheye_camera_info_);

        if (fisheye_rectified_image_publisher_.getNumSubscribers() > 0) {
          cv::remap(fisheye_image_gray, fisheye_image_rect_, cv_warp_map_x_,
                    cv_warp_map_y_, cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);
          sensor_msgs::ImagePtr image_rect = cv_bridge::CvImage(
              cv_bridge_fisheye_image.header, cv_bridge_fisheye_image.encoding,
              fisheye_image_rect_).toImageMsg();
          fisheye_rectified_image_publisher_.publish(image_rect);
        }
      }
    }
  }
}

void TangoRosNode::PublishColorImage() {
  while(ros::ok()) {
    if (!run_threads_) {
      break;
    }
    {
      std::unique_lock<std::mutex> lock(color_image_thread_.data_available_mutex);
      color_image_thread_.data_available.wait(lock);
      if (color_camera_publisher_.getNumSubscribers() > 0 ||
          color_rectified_image_publisher_.getNumSubscribers() > 0) {
        // The Tango image encoding is not supported by ROS.
        // We need to convert it to RGB.
        cv::Mat color_image_rgb;
        cv::cvtColor(color_image_, color_image_rgb, cv::COLOR_YUV420sp2RGBA);
        cv_bridge::CvImage cv_bridge_color_image;
        cv_bridge_color_image.header = color_image_header_;
        cv_bridge_color_image.encoding = sensor_msgs::image_encodings::RGBA8;
        cv_bridge_color_image.image = color_image_rgb;
        color_camera_info_.header = color_image_header_;
        color_camera_info_manager_->setCameraInfo(color_camera_info_);
        sensor_msgs::Image color_image_msg;
        cv_bridge_color_image.toImageMsg(color_image_msg);
        color_camera_publisher_.publish(color_image_msg, color_camera_info_);

        if (color_rectified_image_publisher_.getNumSubscribers() > 0) {
          color_camera_model_.rectifyImage(color_image_rgb, color_image_rect_);
          sensor_msgs::ImagePtr image_rect = cv_bridge::CvImage(
              cv_bridge_color_image.header, cv_bridge_color_image.encoding,
              color_image_rect_).toImageMsg();
          color_rectified_image_publisher_.publish(image_rect);
        }
      }
    }
  }
}

void TangoRosNode::PublishMesh() {
  while(ros::ok()) {
    if (!run_threads_) {
      break;
    }
    if (enable_3dr_mesh_ || enable_3dr_occupancy_grid_) {
      Tango3DR_GridIndexArray t3dr_updated_indices;
      // Update Tango mesh with latest point cloud and color image.
      {
        std::unique_lock<std::mutex> lock(mesh_thread_.data_available_mutex);
        mesh_thread_.data_available.wait(lock);
        tango_3d_reconstruction_helper::UpdateMesh(
            t3dr_context_, point_cloud_manager_, image_buffer_manager_,
            &last_camera_depth_pose_, &last_camera_color_pose_,
            &t3dr_updated_indices);
      }
      // Publish Tango mesh as visualization marker.
      if (enable_3dr_mesh_) {
        visualization_msgs::MarkerArray mesh_marker_array;
        tango_3d_reconstruction_helper::ExtractMeshAndConvertToMarkerArray(
            t3dr_context_, t3dr_updated_indices, time_offset_,
            start_of_service_frame_id_, &mesh_marker_array);
        mesh_marker_publisher_.publish(mesh_marker_array);
        Tango3DR_Status result = Tango3DR_GridIndexArray_destroy(
            &t3dr_updated_indices);
        if (result != TANGO_3DR_SUCCESS) {
          LOG(ERROR) << "Tango3DR_GridIndexArray_destroy failed with error code: "
              << result;
        }
        if (mesh_marker_array.markers.empty()) {
          LOG(INFO) << "Empty mesh array!";
        }
      }
      // Publish Tango mesh as occupancy grid.
      if (enable_3dr_occupancy_grid_) {
        occupancy_grid_.data.clear();
        if (tango_3d_reconstruction_helper::ExtractFloorPlanImageAndConvertToOccupancyGrid(
            t3dr_context_, time_offset_, start_of_service_frame_id_,
            t3dr_resolution_, t3dr_occupancy_grid_threshold_, &occupancy_grid_))
          occupancy_grid_publisher_.publish(occupancy_grid_);
      }
    }
  }
}

void TangoRosNode::DynamicReconfigureCallback(PublisherConfig &config, uint32_t level) {
  laser_scan_max_height_ = config.laser_scan_max_height;
  laser_scan_min_height_ = config.laser_scan_min_height;
}

void TangoRosNode::RunRosSpin() {
  dynamic_reconfigure::Server<tango_ros_native::PublisherConfig> server;
  dynamic_reconfigure::Server<tango_ros_native::PublisherConfig>::CallbackType callback =
      boost::bind(&TangoRosNode::DynamicReconfigureCallback, this, _1, _2);
  server.setCallback(callback);
  while(ros::ok()) {
    if (!run_threads_) {
      break;
    }
    ros::spinOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

bool TangoRosNode::TangoConnectServiceCallback(
    const tango_ros_messages::TangoConnect::Request& request,
    tango_ros_messages::TangoConnect::Response& response) {
  switch (request.request) {
    case tango_ros_messages::TangoConnect::Request::CONNECT:
      // Connect to Tango Service.
      response.response = ConnectToTangoAndSetUpNode();
      break;
    case tango_ros_messages::TangoConnect::Request::DISCONNECT:
      // Disconnect from Tango Service.
      TangoDisconnect();
      response.response = tango_ros_messages::TangoConnect::Response::TANGO_SUCCESS;
      break;
    case tango_ros_messages::TangoConnect::Request::RECONNECT:
      // Disconnect and reconnect to Tango Service.
      TangoDisconnect();
      response.response = ConnectToTangoAndSetUpNode();
      break;
    default:
      LOG(ERROR) << "Did not understand request " << static_cast<int>(request.request)
                 << ", valid requests are (CONNECT: "
                 << tango_ros_messages::TangoConnect::Request::CONNECT
                 << ", DISCONNECT: "
                 << tango_ros_messages::TangoConnect::Request::DISCONNECT
                 << ", RECONNECT: "
                 << tango_ros_messages::TangoConnect::Request::RECONNECT
                 << ")";
      response.message = "Did not understand request.";
      return true;
    }
    return true;
}

bool TangoRosNode::GetMapNameServiceCallback(
    const tango_ros_messages::GetMapName::Request &req,
    tango_ros_messages::GetMapName::Response &res) {
  return GetMapNameFromUuid(req.map_uuid, res.map_name);
}

bool TangoRosNode::GetMapUuidsServiceCallback(
    const tango_ros_messages::GetMapUuids::Request& req,
    tango_ros_messages::GetMapUuids::Response& res) {
  if (!GetAvailableMapUuidsList(res.map_uuids) ) return false;

  res.map_names.resize(res.map_uuids.size());
  auto map_uuids_it = res.map_uuids.begin();
  auto map_names_it = res.map_names.begin();
  for (; map_uuids_it != res.map_uuids.end() && map_names_it != res.map_names.end();
       ++map_uuids_it, ++map_names_it) {
    if (!GetMapNameFromUuid(*map_uuids_it, *map_names_it)) return false;
  }
  return true;
}

bool TangoRosNode::SaveMapServiceCallback(
    const tango_ros_messages::SaveMap::Request& req,
    tango_ros_messages::SaveMap::Response& res) {
  bool save_localization_map = req.request & tango_ros_messages::SaveMap::Request::SAVE_LOCALIZATION_MAP;
  bool save_occupancy_grid = req.request & tango_ros_messages::SaveMap::Request::SAVE_OCCUPANCY_GRID;
  res.message = "";
  if (save_localization_map) {
    res.localization_map_name = req.map_name;
    if (!SaveTangoAreaDescription(res.localization_map_name, res.localization_map_uuid, res.message)) {
      res.success = false;
      return true;
    }
    tango_data_available_ = false;
    res.message += "\nLocalization map " + res.localization_map_uuid +
        " successfully saved with name " + res.localization_map_name;
  } else if (save_occupancy_grid) {
    if (!GetCurrentADFUuid(tango_config_, res.localization_map_uuid)) {
      res.message += "\nCould not get current localization map uuid.";
      res.success = false;
      return true;
    }
  }
  if (save_occupancy_grid) {
    std::string occupancy_grid_directory;
    node_handle_.param(OCCUPANCY_GRID_DIRECTORY_PARAM_NAME,
                       occupancy_grid_directory, OCCUPANCY_GRID_DEFAULT_DIRECTORY);
    res.occupancy_grid_name = req.map_name;
    if (!occupancy_grid_file_io::SaveOccupancyGridToFiles(
        res.occupancy_grid_name, res.localization_map_uuid, occupancy_grid_directory, occupancy_grid_)) {
      res.message += "\nCould not save occupancy grid " + res.occupancy_grid_name
          + " in directory " + occupancy_grid_directory;
      res.success = false;
      return true;
    }
    res.message += "\nOccupancy grid successfully saved with name "
        + res.occupancy_grid_name + " in  directory " + occupancy_grid_directory;
    if (res.localization_map_uuid.empty()) {
      res.message += "\nThe occupancy grid has been saved without localization map uuid. "
          "This means it will not be aligned when loaded later.";
    }
  }
  res.success = true;
  return true;
}

bool TangoRosNode::LoadOccupancyGridServiceCallback(const tango_ros_messages::LoadOccupancyGrid::Request& req,
                           tango_ros_messages::LoadOccupancyGrid::Response& res) {
  nav_msgs::OccupancyGrid occupancy_grid;
  std::string occupancy_grid_directory;
  node_handle_.param(OCCUPANCY_GRID_DIRECTORY_PARAM_NAME,
                     occupancy_grid_directory, OCCUPANCY_GRID_DEFAULT_DIRECTORY);

  std::string map_uuid;
  res.aligned = true;
  if(!occupancy_grid_file_io::LoadOccupancyGridFromFiles(
      req.name, occupancy_grid_directory, &occupancy_grid, &map_uuid)) {
    LOG(ERROR) << "Error while loading occupancy grid from file.";
    res.message =  "Could not load occupancy grid from file " + req.name
              + " in directory " + occupancy_grid_directory;
    res.aligned = false;
    res.success = false;
    return true;
  }
  res.message = "Occupancy grid " + req.name + " successfully loaded from " + occupancy_grid_directory;

  std::string current_map_uuid;
  if (!GetCurrentADFUuid(tango_config_, current_map_uuid)) {
    res.message += "\nCould not get current localization map uuid.";
    res.aligned = false;
    res.success = false;
    return true;
  }
  if (current_map_uuid.empty()) {
    res.message += "\nThe occupancy grid is not aligned because "
        "no localization map is currently used.";
    res.aligned = false;
  }
  if (map_uuid.empty()) {
    res.message += "\nThe occupancy grid is not aligned because "
        "its localization map uuid is empty.";
    res.aligned = false;
  }
  if (map_uuid.compare(current_map_uuid) != 0) {
    res.message += "\nThe occupancy grid is not aligned because "
        "it does not correspond to the localization map currently used.";
    res.aligned = false;
  }

  res.success = true;
  occupancy_grid.header.frame_id = tango_ros_conversions_helper::toFrameId(TANGO_COORDINATE_FRAME_START_OF_SERVICE);
  occupancy_grid.header.stamp = ros::Time::now();
  occupancy_grid.info.map_load_time = occupancy_grid.header.stamp;
  static_occupancy_grid_publisher_.publish(occupancy_grid);
  return true;
}

bool TangoRosNode::GetAvailableMapUuidsList(std::vector<std::string>& uuid_list) {
  char* c_uuid_list;
  TangoErrorType result = TangoService_getAreaDescriptionUUIDList(&c_uuid_list);
  if (result != TANGO_SUCCESS) {
    LOG(ERROR) << "Error while retrieving all available map UUIDs, error: " << result;
    return false;
  }
  if (c_uuid_list == NULL || c_uuid_list[0] == '\0') {
    LOG(WARNING) << "No area description file available.";
    return false;
  }
  LOG(INFO) << "UUID list: " << c_uuid_list;
  uuid_list = SplitCommaSeparatedString(std::string(c_uuid_list));
  return true;
}

bool TangoRosNode::GetMapNameFromUuid(const std::string& map_uuid, std::string& map_name) {
  size_t size = 0;
  char* value;
  TangoAreaDescriptionMetadata metadata;
  TangoErrorType result = TangoService_getAreaDescriptionMetadata(map_uuid.c_str(), &metadata);
  if (result != TANGO_SUCCESS) {
    LOG(ERROR) << "Error while trying to access area description metadata, error: " << result;
    return false;
  }
  result = TangoAreaDescriptionMetadata_get(metadata, "name", &size, &value);
  if (result != TANGO_SUCCESS) {
    LOG(ERROR) << "Error while trying to get area description metadata, error: " << result;
    return false;
  }
  map_name = std::string(value);
  result = TangoAreaDescriptionMetadata_free(metadata);
  if (result != TANGO_SUCCESS) {
    LOG(ERROR) << "Error while trying to free area description metadata, error: " << result;
  }
  LOG(INFO) << "Successfully retrieved map name: " << map_name << " from uuid " << map_uuid;
  return true;
}
} // namespace tango_ros_native
