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
std::vector<std::string> splitCommaSeparatedString(const std::string& comma_separated_string) {
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
void onPoseAvailableRouter(void* context, const TangoPoseData* pose) {
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
void onPointCloudAvailableRouter(void* context,
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
void onFrameAvailableRouter(void* context, TangoCameraId camera_id,
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
std::string getCurrentDateAndTime() {
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
  point_cloud_publisher_ =
      node_handle_.advertise<sensor_msgs::PointCloud2>(
          POINT_CLOUD_TOPIC_NAME, queue_size, latching);
  laser_scan_publisher_ =
      node_handle_.advertise<sensor_msgs::LaserScan>(
          LASER_SCAN_TOPIC_NAME, queue_size, latching);

  image_transport_.reset(new image_transport::ImageTransport(node_handle_));
  try {
    fisheye_camera_publisher_ =
        image_transport_->advertiseCamera(FISHEYE_IMAGE_TOPIC_NAME,
                                          queue_size, latching);
    fisheye_rectified_image_publisher_ =
        image_transport_->advertise(FISHEYE_RECTIFIED_IMAGE_TOPIC_NAME,
                                   queue_size, latching);
    color_camera_publisher_ =
        image_transport_->advertiseCamera(COLOR_IMAGE_TOPIC_NAME,
                                          queue_size, latching);
    color_rectified_image_publisher_ =
        image_transport_->advertise(COLOR_RECTIFIED_IMAGE_TOPIC_NAME,
                                   queue_size, latching);
  } catch (const image_transport::Exception& e) {
    LOG(ERROR) << "Error while creating image transport publishers" << e.what();
  }

  mesh_marker_publisher_ =
      node_handle_.advertise<visualization_msgs::MarkerArray>(
          COLOR_MESH_TOPIC_NAME, queue_size, latching);

  get_map_name_service_ = node_handle_.advertiseService<tango_ros_messages::GetMapName::Request,
      tango_ros_messages::GetMapName::Response>(GET_MAP_NAME_SERVICE_NAME,
                                             boost::bind(&TangoRosNode::GetMapName, this, _1, _2));

  get_map_uuids_service_ = node_handle_.advertiseService<tango_ros_messages::GetMapUuids::Request,
      tango_ros_messages::GetMapUuids::Response>(GET_MAP_UUIDS_SERVICE_NAME,
                                             boost::bind(&TangoRosNode::GetMapUuids, this, _1, _2));

  save_map_service_ = node_handle_.advertiseService<tango_ros_messages::SaveMap::Request,
      tango_ros_messages::SaveMap::Response>(SAVE_MAP_SERVICE_NAME,
                                             boost::bind(&TangoRosNode::SaveMap, this, _1, _2));

  tango_connect_service_ = node_handle_.advertiseService<tango_ros_messages::TangoConnect::Request,
          tango_ros_messages::TangoConnect::Response>(
              CONNECT_SERVICE_NAME, boost::bind(
                  &TangoRosNode::TangoConnectServiceCallback, this, _1, _2));

  request_permission_service_ = node_handle_.serviceClient<tango_ros_messages::RequestPermission>(REQUEST_PERMISSION_SERVICE_NAME);

  tango_status_ = TangoStatus::UNKNOWN;

  if (!node_handle_.hasParam(CREATE_NEW_MAP_PARAM_NAME)) {
    node_handle_.setParam(CREATE_NEW_MAP_PARAM_NAME, false);
  }
  if (!node_handle_.hasParam(LOCALIZATION_MODE_PARAM_NAME)) {
    node_handle_.setParam(LOCALIZATION_MODE_PARAM_NAME, 2);
  }
  if (!node_handle_.hasParam(LOCALIZATION_MAP_UUID_PARAM_NAME)) {
    node_handle_.setParam(LOCALIZATION_MAP_UUID_PARAM_NAME, "");
  }
  if (!node_handle_.hasParam(DATASET_PATH_PARAM_NAME)) {
    node_handle_.setParam(DATASET_PATH_PARAM_NAME, DEFAULT_DATASETS_PATH);
  }
  if (!node_handle_.hasParam(DATASET_UUID_PARAM_NAME)) {
    node_handle_.setParam(DATASET_UUID_PARAM_NAME, "");
  }
  if (node_handle_.hasParam(PUBLISH_POSE_ON_TF_PARAM_NAME)) {
    node_handle_.getParam(PUBLISH_POSE_ON_TF_PARAM_NAME, publish_pose_on_tf_);
  } else {
    node_handle_.setParam(PUBLISH_POSE_ON_TF_PARAM_NAME, true);
  }
  if (node_handle_.hasParam(ENABLE_DEPTH)) {
    node_handle_.param(ENABLE_DEPTH, enable_depth_, true);
  }
  if (node_handle_.hasParam(ENABLE_COLOR_CAMERA)) {
    node_handle_.param(ENABLE_COLOR_CAMERA, enable_color_camera_, true);
  }
}

TangoRosNode::~TangoRosNode() {
  TangoDisconnect();
}

TangoErrorType TangoRosNode::OnTangoServiceConnected() {
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
  time_offset_ =  ros::Time::now().toSec() - pose.timestamp;

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

  tango_ros_conversions_helper::toTango3DR_CameraCalibration(
      tango_camera_intrinsics, &t3dr_color_camera_intrinsics_);
  TangoSetup3DRConfig();
  return TANGO_SUCCESS;
}

void TangoRosNode::RequestADFPermission() {
  tango_ros_messages::RequestPermission srv;
  srv.request.permission = tango_ros_messages::RequestPermission::Request::ADF_PERMISSION;
  if (request_permission_service_.call(srv)) {
    if (srv.response.granted) {
      LOG(INFO) << "ADF permission has been requested and granted by the user.";
    } else {
      LOG(WARNING) << "ADF permission has been requested but not granted by the user.";
    }
  } else {
    LOG(ERROR) << "Failed to call service to request ADF permission";
  }
}

void TangoRosNode::RequestDatasetPermission() {
  tango_ros_messages::RequestPermission srv;
  srv.request.permission = tango_ros_messages::RequestPermission::Request::DATASET_PERMISSION;
  if (request_permission_service_.call(srv)) {
    if (srv.response.granted) {
      LOG(INFO) << "Dataset permission has been requested and granted by the user.";
    } else {
      LOG(WARNING) << "Dataset permission has been requested but not granted by the user.";
    }
  } else {
    LOG(ERROR) << "Failed to call service to request dataset permission";
  }
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
  node_handle_.param<bool>(ENABLE_DEPTH, enable_depth_, true);
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
  node_handle_.param<bool>(ENABLE_COLOR_CAMERA, enable_color_camera_, true);
  result = TangoConfig_setBool(tango_config_, config_enable_color_camera, enable_color_camera_);
  if (result != TANGO_SUCCESS) {
    LOG(ERROR) << function_name << ", TangoConfig_setBool "
        << config_enable_color_camera << " error: " << result;
    return result;
  }
  std::string datasets_path;
  node_handle_.param(DATASET_PATH_PARAM_NAME, datasets_path, DEFAULT_DATASETS_PATH);
  const char* config_datasets_path = "config_datasets_path";
  result = TangoConfig_setString(tango_config_, config_datasets_path, datasets_path.c_str());
  if (result != TANGO_SUCCESS) {
    LOG(ERROR) << function_name << ", TangoConfig_setString "
               << config_datasets_path << " error: " << result;
    return result;
  }
  std::string dataset_uuid;
  node_handle_.param(DATASET_UUID_PARAM_NAME, dataset_uuid, std::string(""));
  if (dataset_uuid != "") {
    RequestDatasetPermission();
    const char* config_experimental_load_dataset_UUID = "config_experimental_load_dataset_UUID";
    result = TangoConfig_setString(tango_config_, config_experimental_load_dataset_UUID, dataset_uuid.c_str());
    if (result != TANGO_SUCCESS) {
      LOG(ERROR) << function_name << ", TangoConfig_setString "
                 << config_experimental_load_dataset_UUID << " error: " << result;
      return result;
    }
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

Tango3DR_Status TangoRosNode::TangoSetup3DRConfig() {
  const char* function_name = "TangoRosNode::TangoSetup3DRConfig()";

  Tango3DR_ConfigH t3dr_config =
      Tango3DR_Config_create(TANGO_3DR_CONFIG_CONTEXT);
  Tango3DR_Status result;
  const char* resolution = "resolution";
  result = Tango3DR_Config_setDouble(t3dr_config, resolution, 0.05);
  if (result != TANGO_3DR_SUCCESS) {
    LOG(ERROR) << function_name << ", Tango3DR_Config_setDouble "
        << resolution << " error: " << result;
    return result;
  }
  const char* generate_color = "generate_color";
  result = Tango3DR_Config_setBool(t3dr_config, generate_color, true);
  if (result != TANGO_3DR_SUCCESS) {
    LOG(ERROR) << function_name << ", Tango3DR_Config_setBool "
        << generate_color << " error: " << result;
    return result;
  }
  const char* use_floorplan = "use_floorplan";
  node_handle_.param(USE_FLOOR_PLAN_PARAM_NAME, use_floor_plan_, false);
  result = Tango3DR_Config_setBool(t3dr_config, use_floorplan, use_floor_plan_);
  if (result != TANGO_3DR_SUCCESS) {
    LOG(ERROR) << function_name << ", Tango3DR_Config_setBool "
        << use_floorplan << " error: " << result;
    return result;
  }
  t3dr_context_ = Tango3DR_create(t3dr_config);
  if (t3dr_context_ == nullptr) {
    LOG(ERROR) << function_name << ", Tango3DR_create error: Unable to create 3DR context.";
    return TANGO_3DR_ERROR;
  }
  // Configure the color camera intrinsics to be used with updates to the mesh.
  result = Tango3DR_setColorCalibration(t3dr_context_, &t3dr_color_camera_intrinsics_);
  if (t3dr_context_ == nullptr) {
    LOG(ERROR) << function_name << ", Unable to set color calibration.";
    return TANGO_3DR_ERROR;
  }
  return Tango3DR_Config_destroy(t3dr_config);
}

TangoErrorType TangoRosNode::TangoConnect() {
  const char* function_name = "TangoRosNode::TangoConnect()";

  TangoCoordinateFramePair pair;
  pair.base = TANGO_COORDINATE_FRAME_START_OF_SERVICE;
  pair.target = TANGO_COORDINATE_FRAME_DEVICE;

  TangoErrorType result;
  result = TangoService_connectOnPoseAvailable(1, &pair, onPoseAvailableRouter);
  if (result != TANGO_SUCCESS) {
    LOG(ERROR) << function_name
        << ", TangoService_connectOnPoseAvailable error: " << result;
    return result;
  }

  result = TangoService_connectOnPointCloudAvailable(onPointCloudAvailableRouter);
  if (result != TANGO_SUCCESS) {
    LOG(ERROR) << function_name
        << ", TangoService_connectOnPointCloudAvailable error: " << result;
    return result;
  }

  result = TangoService_connectOnFrameAvailable(
      TANGO_CAMERA_FISHEYE, this, onFrameAvailableRouter);
  if (result != TANGO_SUCCESS) {
    LOG(ERROR) << function_name
        << ", TangoService_connectOnFrameAvailable TANGO_CAMERA_FISHEYE error: " << result;
    return result;
  }

  result = TangoService_connectOnFrameAvailable(
      TANGO_CAMERA_COLOR, this, onFrameAvailableRouter);
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
  PublishStaticTransforms();
  success = OnTangoServiceConnected();
  if (success != TANGO_SUCCESS) {
    UpdateAndPublishTangoStatus(TangoStatus::NO_FIRST_VALID_POSE);
    return success;
  }
  // Create publishing threads.
  StartPublishing();
  UpdateAndPublishTangoStatus(TangoStatus::SERVICE_CONNECTED);
  tango_data_available_ = true;
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
    Tango3DR_destroy(t3dr_context_);
    t3dr_context_ = nullptr;
  }
  TangoService_disconnect();
  UpdateAndPublishTangoStatus(TangoStatus::SERVICE_NOT_CONNECTED);
}

void TangoRosNode::PublishStaticTransforms() {
  TangoCoordinateFramePair pair;
  TangoPoseData pose;

  pair.base = TANGO_COORDINATE_FRAME_DEVICE;
  pair.target = TANGO_COORDINATE_FRAME_IMU;
  TangoService_getPoseAtTime(0.0, pair, &pose);
  geometry_msgs::TransformStamped device_T_imu;
  tango_ros_conversions_helper::toTransformStamped(pose, time_offset_, &device_T_imu);
  tf_static_broadcaster_.sendTransform(device_T_imu);

  pair.base = TANGO_COORDINATE_FRAME_DEVICE;
  pair.target = TANGO_COORDINATE_FRAME_CAMERA_DEPTH;
  TangoService_getPoseAtTime(0.0, pair, &pose);
  tango_ros_conversions_helper::toTransformStamped(pose, time_offset_, &device_T_camera_depth_);
  tf_static_broadcaster_.sendTransform(device_T_camera_depth_);

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
  tf_static_broadcaster_.sendTransform(camera_depth_T_laser_message);

  pair.base = TANGO_COORDINATE_FRAME_DEVICE;
  pair.target = TANGO_COORDINATE_FRAME_CAMERA_FISHEYE;
  TangoService_getPoseAtTime(0.0, pair, &pose);
  tango_ros_conversions_helper::toTransformStamped(pose, time_offset_,
                                           &device_T_camera_fisheye_);
  tf_static_broadcaster_.sendTransform(device_T_camera_fisheye_);

  pair.base = TANGO_COORDINATE_FRAME_DEVICE;
  pair.target = TANGO_COORDINATE_FRAME_CAMERA_COLOR;
  TangoService_getPoseAtTime(0.0, pair, &pose);
  tango_ros_conversions_helper::toTransformStamped(pose, time_offset_,
                                           &device_T_camera_color_);
  tf_static_broadcaster_.sendTransform(device_T_camera_color_);
}

void TangoRosNode::OnPoseAvailable(const TangoPoseData* pose) {
  if (publish_pose_on_tf_ ||
      start_of_service_T_device_publisher_.getNumSubscribers() > 0 ||
      area_description_T_start_of_service_publisher_.getNumSubscribers() > 0) {
    if (pose->frame.base == TANGO_COORDINATE_FRAME_START_OF_SERVICE &&
        pose->frame.target == TANGO_COORDINATE_FRAME_DEVICE) {
      if (pose->status_code == TANGO_POSE_VALID &&
          device_pose_thread_.data_available_mutex.try_lock()) {
        tango_ros_conversions_helper::toTransformStamped(*pose, time_offset_,
                                                 &start_of_service_T_device_);
        TangoCoordinateFramePair pair;
        pair.base = TANGO_COORDINATE_FRAME_AREA_DESCRIPTION;
        pair.target = TANGO_COORDINATE_FRAME_START_OF_SERVICE;
        TangoPoseData area_description_T_start_of_service;
        TangoService_getPoseAtTime(0.0, pair, &area_description_T_start_of_service);
        if (area_description_T_start_of_service.status_code == TANGO_POSE_VALID) {
          tango_ros_conversions_helper::toTransformStamped(area_description_T_start_of_service,
                             time_offset_, &area_description_T_start_of_service_);
        }
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

    if (mesh_marker_publisher_.getNumSubscribers() > 0) {
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

  if (mesh_marker_publisher_.getNumSubscribers() > 0 &&
      camera_id == TangoCameraId::TANGO_CAMERA_COLOR &&
      new_point_cloud_available_for_t3dr_ &&
      mesh_marker_thread_.data_available_mutex.try_lock()) {
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
    mesh_marker_thread_.data_available_mutex.unlock();
    mesh_marker_thread_.data_available.notify_all();
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
  mesh_marker_thread_.publish_thread =
      std::thread(&TangoRosNode::PublishMeshMarker, this);
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
    if (mesh_marker_thread_.publish_thread.joinable()) {
      if (!tango_data_available_ || !enable_depth_ || !enable_color_camera_ ||
          mesh_marker_publisher_.getNumSubscribers() <= 0) {
        mesh_marker_thread_.data_available.notify_all();
      }
      mesh_marker_thread_.publish_thread.join();
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

void TangoRosNode::PublishMeshMarker() {
  while(ros::ok()) {
    if (!run_threads_) {
      break;
    }
    if (mesh_marker_publisher_.getNumSubscribers() > 0) {
      Tango3DR_GridIndexArray* t3dr_updated_indices;
      {
        std::unique_lock<std::mutex> lock(mesh_marker_thread_.data_available_mutex);
        mesh_marker_thread_.data_available.wait(lock);
        // Get latest point cloud.
        TangoPointCloud* last_point_cloud;
        TangoSupport_getLatestPointCloud(point_cloud_manager_, &last_point_cloud);
        Tango3DR_PointCloud t3dr_depth;
        t3dr_depth.timestamp = last_point_cloud->timestamp;
        t3dr_depth.num_points = last_point_cloud->num_points;
        t3dr_depth.points = reinterpret_cast<Tango3DR_Vector4*>(last_point_cloud->points);
        // Get latest image.
        TangoImageBuffer* last_color_image_buffer;
        if (image_buffer_manager_ != nullptr) {
          TangoSupport_getLatestImageBuffer(image_buffer_manager_, &last_color_image_buffer);
          Tango3DR_ImageBuffer t3dr_image;
          t3dr_image.width = last_color_image_buffer->width;
          t3dr_image.height = last_color_image_buffer->height;
          t3dr_image.stride = last_color_image_buffer->stride;
          t3dr_image.timestamp = last_color_image_buffer->timestamp;
          t3dr_image.format = static_cast<Tango3DR_ImageFormatType>(last_color_image_buffer->format);
          t3dr_image.data = last_color_image_buffer->data;
          // Get updated mesh segment indices.
          Tango3DR_Status result =
              Tango3DR_update(t3dr_context_, &t3dr_depth, &last_camera_depth_pose_, &t3dr_image,
                              &last_camera_color_pose_, &t3dr_updated_indices);
          if (result != TANGO_3DR_SUCCESS) {
            LOG(ERROR) << "Tango3DR_update failed with error code " << result;
          }
        }
      }
      if (t3dr_updated_indices == nullptr) {
        LOG(INFO) << "No indices updated in mesh";
        continue;
      }
      visualization_msgs::MarkerArray mesh_marker_array;
      for (size_t i = 0; i < t3dr_updated_indices->num_indices; ++i) {
        // Extract tango mesh from updated index.
        Tango3DR_Mesh* tango_mesh;
        if(Tango3DR_extractMeshSegment(
            t3dr_context_, t3dr_updated_indices->indices[i], &tango_mesh) != TANGO_3DR_SUCCESS) {
          LOG(ERROR) << "Tango3DR_extractMeshSegment failed";
        }
        if (tango_mesh == nullptr || tango_mesh->num_faces == 0) {
          LOG(INFO) << "Invalid mesh extracted!";
          continue;
        }
        // Make mesh marker from tango mesh.
        visualization_msgs::Marker mesh_marker;
        tango_ros_conversions_helper::toMeshMarker(t3dr_updated_indices->indices[i],
                                           tango_mesh, time_offset_, &mesh_marker);
        // Free tango mesh once we are finished with it.
        Tango3DR_Status result = Tango3DR_Mesh_destroy(tango_mesh);
        if (result != TANGO_3DR_SUCCESS) {
          LOG(ERROR) << "Tango3DR_Mesh_destroy error: " << result;
        }
        if (mesh_marker.points.empty()) {
          LOG(INFO) << "Empty mesh marker.";
          continue;
        }
        mesh_marker_array.markers.push_back(mesh_marker);
      }
      Tango3DR_Status result = Tango3DR_GridIndexArray_destroy(t3dr_updated_indices);
      if (result != TANGO_3DR_SUCCESS) {
        LOG(ERROR) << "Tango3DR_GridIndexArray_destroy error: " << result;
      }
      if (mesh_marker_array.markers.empty()) {
        LOG(INFO) << "Empty mesh array!";
      }
      mesh_marker_publisher_.publish(mesh_marker_array);
    }
  }
}

void TangoRosNode::DynamicReconfigureCallback(PublisherConfig &config, uint32_t level) {
  laser_scan_max_height_ = config.laser_scan_max_height;
  laser_scan_min_height_ = config.laser_scan_min_height;
  if (config.use_floor_plan != use_floor_plan_) {
    use_floor_plan_ = config.use_floor_plan;
    const char* use_floorplan = "use_floorplan";
    Tango3DR_Status result = Tango3DR_setRuntimeBool(t3dr_context_, use_floorplan, use_floor_plan_);
    if (result != TANGO_3DR_SUCCESS) {
      LOG(ERROR) << "Failed to change runtime config " << use_floorplan << " error: " << result;
    }

  }
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
      return false;
    }
    return true;
}

bool TangoRosNode::GetMapName(
    const tango_ros_messages::GetMapName::Request &req,
    tango_ros_messages::GetMapName::Response &res) {
  res.map_name = GetMapNameFromUuid(req.map_uuid);
}

bool TangoRosNode::GetMapUuids(
    const tango_ros_messages::GetMapUuids::Request &req,
    tango_ros_messages::GetMapUuids::Response &res) {
  res.map_uuids = splitCommaSeparatedString(GetAvailableMapUuidsList());

  for (const std::string uuid : res.map_uuids) {
    res.map_names.push_back(GetMapNameFromUuid(uuid));
  }
}

bool TangoRosNode::SaveMap(tango_ros_messages::SaveMap::Request &req,
                           tango_ros_messages::SaveMap::Response &res) {
  RequestADFPermission();
  TangoErrorType result;
  TangoUUID map_uuid;
  result = TangoService_saveAreaDescription(&map_uuid);
  if (result != TANGO_SUCCESS) {
    LOG(ERROR) << "Error while saving area description, error: " << result;
    res.message =  "Could not save the map. Did you turn on create_new_map? "
        "Did you allow the app to use area learning?";
    res.success = false;
    return true;
  }
  TangoAreaDescriptionMetadata metadata;
  result = TangoService_getAreaDescriptionMetadata(map_uuid, &metadata);
  if (result != TANGO_SUCCESS) {
    LOG(ERROR) << "Error while trying to access area description metadata, error: " << result;
    res.message =  "Could not access map metadata";
    res.success = false;
    return true;
  }
  // Prepend name with date and time.
  std::string map_name = getCurrentDateAndTime() + " " + req.map_name;
  result = TangoAreaDescriptionMetadata_set(metadata, "name", map_name.capacity(), map_name.c_str());
  if (result != TANGO_SUCCESS) {
    LOG(ERROR) << "Error while trying to change area description metadata, error: " << result;
    res.message =  "Could not set the name of the map";
    res.success = false;
    return true;
  }
  result = TangoService_saveAreaDescriptionMetadata(map_uuid, metadata);
  if (result != TANGO_SUCCESS) {
    LOG(ERROR) << "Error while saving new area description metadata, error: " << result;
    res.message =  "Could not save map metadata";
    res.success = false;
    return true;
  }
  result = TangoAreaDescriptionMetadata_free(metadata);
  if (result != TANGO_SUCCESS) {
    LOG(ERROR) << "Error while trying to free area description metadata, error: " << result;
    res.message =  "Could not free map metadata";
    res.success = false;
    return true;
  }

  std::string map_uuid_string = static_cast<std::string>(map_uuid);
  res.message =  "Map " + map_uuid_string + " successfully saved with the following name: " + map_name;
  res.map_name = map_name;
  res.map_uuid = map_uuid_string;
  res.success = true;
  tango_data_available_ = false;
  return true;
}

std::string TangoRosNode::GetAvailableMapUuidsList() {
  RequestADFPermission();
  char* uuid_list;
  TangoErrorType result = TangoService_getAreaDescriptionUUIDList(&uuid_list);
  if (result != TANGO_SUCCESS) {
    LOG(ERROR) << "Error while retrieving all available map UUIDs, error: " << result;
    return "";
  }
  if (uuid_list != NULL && uuid_list[0] != '\0') {
    LOG(INFO) << "UUID list: " << uuid_list;
  } else {
    LOG(ERROR) << "No area description file available.";
    return "";
  }
  return std::string(uuid_list);
}

std::string TangoRosNode::GetMapNameFromUuid(const std::string& map_uuid) {
  RequestADFPermission();
  size_t size = 0;
  char* value;
  TangoAreaDescriptionMetadata metadata;
  TangoErrorType result = TangoService_getAreaDescriptionMetadata(map_uuid.c_str(), &metadata);
  if (result != TANGO_SUCCESS) {
    LOG(ERROR) << "Error while trying to access area description metadata, error: " << result;
  }
  result = TangoAreaDescriptionMetadata_get(metadata, "name", &size, &value);
  if (result != TANGO_SUCCESS) {
    LOG(ERROR) << "Error while trying to get area description metadata, error: " << result;
  }
  std::string map_name = std::string(value);
  result = TangoAreaDescriptionMetadata_free(metadata);
  if (result != TANGO_SUCCESS) {
    LOG(ERROR) << "Error while trying to free area description metadata, error: " << result;
  }
  LOG(INFO) << "Successfully retrieved map name: " << map_name << " from uuid " << map_uuid;
  return map_name;
}
} // namespace tango_ros_native
