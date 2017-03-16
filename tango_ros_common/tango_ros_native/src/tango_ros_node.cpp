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
#include "tango_ros_native/tango_ros_node.h"

#include <cmath>

#include <glog/logging.h>

#include <dynamic_reconfigure/config_tools.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace {
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
// Converts a TangoPoseData to a geometry_msgs::TransformStamped.
// @param pose, TangoPoseData to convert.
// @param time_offset, offset in s between pose (tango time) and
//        transform (ros time).
// @param transform, the output TransformStamped.
void toTransformStamped(const TangoPoseData& pose,
                        double time_offset,
                        geometry_msgs::TransformStamped* transform) {
  transform->transform.translation.x = pose.translation[0];
  transform->transform.translation.y = pose.translation[1];
  transform->transform.translation.z = pose.translation[2];
  transform->transform.rotation.x = pose.orientation[0];
  transform->transform.rotation.y = pose.orientation[1];
  transform->transform.rotation.z = pose.orientation[2];
  transform->transform.rotation.w = pose.orientation[3];
  transform->header.stamp.fromSec(pose.timestamp + time_offset);
}
// Converts a TangoPointCloud to a sensor_msgs::PointCloud2.
// @param tango_point_cloud, TangoPointCloud to convert.
// @param time_offset, offset in s between tango_point_cloud (tango time) and
//        point_cloud (ros time).
// @param point_cloud, the output PointCloud2.
void toPointCloud2(const TangoPointCloud& tango_point_cloud,
                   double time_offset,
                   sensor_msgs::PointCloud2* point_cloud) {
  point_cloud->width = tango_point_cloud.num_points;
  point_cloud->height = 1;
  point_cloud->point_step = (sizeof(float) * tango_ros_native::NUMBER_OF_FIELDS_IN_POINT_CLOUD);
  point_cloud->is_dense = true;
  point_cloud->row_step = point_cloud->width;
  point_cloud->is_bigendian = false;
  point_cloud->data.resize(tango_point_cloud.num_points);
  sensor_msgs::PointCloud2Modifier modifier(*point_cloud);
  modifier.setPointCloud2Fields(tango_ros_native::NUMBER_OF_FIELDS_IN_POINT_CLOUD,
                                "x", 1, sensor_msgs::PointField::FLOAT32,
                                "y", 1, sensor_msgs::PointField::FLOAT32,
                                "z", 1, sensor_msgs::PointField::FLOAT32,
                                "c", 1, sensor_msgs::PointField::FLOAT32);
  modifier.resize(tango_point_cloud.num_points);
  sensor_msgs::PointCloud2Iterator<float> iter_x(*point_cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*point_cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*point_cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_c(*point_cloud, "c");
  for (size_t i = 0; i < tango_point_cloud.num_points;
      ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_c) {
    *iter_x = tango_point_cloud.points[i][0];
    *iter_y = tango_point_cloud.points[i][1];
    *iter_z = tango_point_cloud.points[i][2];
    *iter_c = tango_point_cloud.points[i][3];
  }
  point_cloud->header.stamp.fromSec(tango_point_cloud.timestamp + time_offset);
}
// Convert a point to a laser scan range.
// Method taken from the ros package 'pointcloud_to_laserscan':
// http://wiki.ros.org/pointcloud_to_laserscan
// @param x x coordinate of the point in the laser scan frame.
// @param y y coordinate of the point in the laser scan frame.
// @param z z coordinate of the point in the laser scan frame.
// @param min_height minimum height for a point of the point cloud to be
// included in laser scan.
// @param max_height maximum height for a point of the point cloud to be
// included in laser scan.
// @param laser_scan, the output LaserScan containing the range data.
void toLaserScanRange(double x, double y, double z, double min_height,
                      double max_height, sensor_msgs::LaserScan* laser_scan) {
  if (std::isnan(x) || std::isnan(y) || std::isnan(z)) {
    // NAN point.
    return;
  }
  if (z > max_height || z < min_height) {
    // Z not in height range.
    return;
  }
  double range = hypot(x, y);
  if (range < laser_scan->range_min) {
    // Point not in distance range.
    return;
  }
  double angle = atan2(y, x);
  if (angle < laser_scan->angle_min || angle > laser_scan->angle_max) {
    // Point not in angle range.
    return;
  }

  if (range > laser_scan->range_max) {
    laser_scan->range_max = range;
  }
  // Overwrite range at laser scan ray if new range is smaller.
  int index = (angle - laser_scan->angle_min) / laser_scan->angle_increment;
  if (range < laser_scan->ranges[index]) {
    laser_scan->ranges[index] = range;
  }
}
// Converts a TangoPointCloud to a sensor_msgs::LaserScan.
// @param tango_point_cloud, TangoPointCloud to convert.
// @param time_offset, offset in s between tango_point_cloud (tango time) and
//        laser_scan (ros time).
// @param min_height minimum height for a point of the point cloud to be
// included in laser scan.
// @param max_height maximum height for a point of the point cloud to be
// included in laser scan.
// @param point_cloud_T_laser transformation from point cloud to
// laser scan frame.
// @param laser_scan, the output LaserScan.
void toLaserScan(const TangoPointCloud& tango_point_cloud,
                 double time_offset,
                 double min_height,
                 double max_height,
                 const tf::Transform& point_cloud_T_laser,
                 sensor_msgs::LaserScan* laser_scan) {
  for (size_t i = 0; i < tango_point_cloud.num_points; ++i) {
    const tf::Vector3 point_cloud_p(tango_point_cloud.points[i][0],
                                    tango_point_cloud.points[i][1],
                                    tango_point_cloud.points[i][2]);
    tf::Vector3 laser_scan_p  = point_cloud_T_laser.inverse() * point_cloud_p;
    toLaserScanRange(laser_scan_p.getX(), laser_scan_p.getY(), laser_scan_p.getZ(),
                     min_height, max_height, laser_scan);
  }
  laser_scan->header.stamp.fromSec(tango_point_cloud.timestamp + time_offset);
}
// Converts a TangoCoordinateFrameType to a ros frame ID i.e. a string.
// @param tango_frame_type, TangoCoordinateFrameType to convert.
// @return returns the corresponding frame id.
std::string toFrameId(const TangoCoordinateFrameType& tango_frame_type) {
  std::string string_frame_type;
  switch(tango_frame_type) {
    case TANGO_COORDINATE_FRAME_AREA_DESCRIPTION:
      string_frame_type = "area_description";
      break;
    case TANGO_COORDINATE_FRAME_CAMERA_COLOR:
      string_frame_type = "camera_color";
      break;
    case TANGO_COORDINATE_FRAME_CAMERA_DEPTH:
      string_frame_type = "camera_depth";
      break;
    case TANGO_COORDINATE_FRAME_CAMERA_FISHEYE:
      string_frame_type = "camera_fisheye";
      break;
    case TANGO_COORDINATE_FRAME_DEVICE:
      string_frame_type = "device";
      break;
    case TANGO_COORDINATE_FRAME_DISPLAY:
      string_frame_type = "display";
      break;
    case TANGO_COORDINATE_FRAME_GLOBAL_WGS84:
      string_frame_type = "global_wgs84";
      break;
    case TANGO_COORDINATE_FRAME_IMU:
      string_frame_type = "imu";
      break;
    case TANGO_COORDINATE_FRAME_PREVIOUS_DEVICE_POSE:
      string_frame_type = "previous_device_pose";
      break;
    case TANGO_COORDINATE_FRAME_START_OF_SERVICE:
      string_frame_type = "start_of_service";
      break;
    case TANGO_COORDINATE_FRAME_UUID:
      string_frame_type = "uuid";
      break;
    default:
      LOG(ERROR) << "Unknown TangoCoordinateFrameType: " << tango_frame_type;
      string_frame_type = "unknown";
      break;
  }
  return string_frame_type;
}
// Converts TangoCameraIntrinsics to sensor_msgs::CameraInfo.
// See Tango documentation:
// http://developers.google.com/tango/apis/unity/reference/class/tango/tango-camera-intrinsics
// And ROS documentation:
// http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
// @param camera_intrinsics, TangoCameraIntrinsics to convert.
// @param camera_info, the output CameraInfo.
void toCameraInfo(const TangoCameraIntrinsics& camera_intrinsics,
                  sensor_msgs::CameraInfo* camera_info) {
  camera_info->height = camera_intrinsics.height;
  camera_info->width = camera_intrinsics.width;
  camera_info->K = {camera_intrinsics.fx, 0., camera_intrinsics.cx,
                    0., camera_intrinsics.fy, camera_intrinsics.cy,
                    0., 0., 1.};
  camera_info->R = {1., 0., 0., 0., 1., 0., 0., 0., 1.};
  camera_info->P = {camera_intrinsics.fx, 0., camera_intrinsics.cx, 0.,
                    0., camera_intrinsics.fy, camera_intrinsics.cy, 0.,
                    0., 0., 1., 0.};
  if (camera_intrinsics.camera_id == TangoCameraId::TANGO_CAMERA_FISHEYE) {
    camera_info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    camera_info->D = {camera_intrinsics.distortion[0],
        camera_intrinsics.distortion[1], camera_intrinsics.distortion[2],
        camera_intrinsics.distortion[3], camera_intrinsics.distortion[4]};
  } else if (camera_intrinsics.camera_id == TangoCameraId::TANGO_CAMERA_COLOR) {
    camera_info->distortion_model = sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;
    camera_info->D = {camera_intrinsics.distortion[0],
        camera_intrinsics.distortion[1], 0., 0., camera_intrinsics.distortion[2]};
  } else {
    LOG(ERROR) << "Unknown camera ID: " << camera_intrinsics.camera_id;
  }
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
}  // namespace

namespace tango_ros_native {
TangoRosNode::TangoRosNode() : run_threads_(false) {
  const  uint32_t queue_size = 1;
  const bool latching = true;
  point_cloud_publisher_ =
      node_handle_.advertise<sensor_msgs::PointCloud2>(
          publisher_config_.point_cloud_topic, queue_size, latching);
  laser_scan_publisher_ =
      node_handle_.advertise<sensor_msgs::LaserScan>(
          publisher_config_.laser_scan_topic, queue_size, latching);

  image_transport_.reset(new image_transport::ImageTransport(node_handle_));
  try {
    fisheye_camera_publisher_ =
        image_transport_->advertiseCamera(publisher_config_.fisheye_image_topic,
                                          queue_size, latching);
    fisheye_rectified_image_publisher_ =
        image_transport_->advertise(publisher_config_.fisheye_rectified_image_topic,
                                   queue_size, latching);
    color_camera_publisher_ =
        image_transport_->advertiseCamera(publisher_config_.color_image_topic,
                                          queue_size, latching);
    color_rectified_image_publisher_ =
        image_transport_->advertise(publisher_config_.color_rectified_image_topic,
                                   queue_size, latching);
  } catch (const image_transport::Exception& e) {
    LOG(ERROR) << "Error while creating image transport publishers" << e.what();
  }
}

TangoRosNode::TangoRosNode(const PublisherConfiguration& publisher_config) :
    TangoRosNode() {
  publisher_config_.publish_device_pose = static_cast<bool>(publisher_config.publish_device_pose);
  publisher_config_.publish_point_cloud = static_cast<bool>(publisher_config.publish_point_cloud);
  publisher_config_.publish_laser_scan = static_cast<bool>(publisher_config.publish_laser_scan);
  publisher_config_.publish_camera = static_cast<bool>(publisher_config.publish_camera);
}

TangoRosNode::~TangoRosNode() {
  StopPublishing();
  if (tango_config_ != nullptr) {
    TangoConfig_free(tango_config_);
  }
}

TangoErrorType TangoRosNode::OnTangoServiceConnected() {
  TangoErrorType result = TangoSetupConfig();
  if (result != TANGO_SUCCESS) {
      LOG(ERROR) << "Error while setting up Tango config.";
      return result;
  }
  result = TangoConnect();
  if (result != TANGO_SUCCESS) {
    LOG(ERROR) << "Error while connecting to Tango Service.";
    return result;
  }

  PublishStaticTransforms();
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
  toCameraInfo(tango_camera_intrinsics, &fisheye_camera_info_);
  fisheye_camera_info_manager_.reset(new camera_info_manager::CameraInfoManager(node_handle_));
  fisheye_camera_info_manager_->setCameraName("fisheye_1");
  // Cache warp maps for more efficiency.
  cv_warp_map_x_.create(fisheye_camera_info_.height, fisheye_camera_info_.width, CV_32FC1);
  cv_warp_map_y_.create(fisheye_camera_info_.height, fisheye_camera_info_.width, CV_32FC1);
  ComputeWarpMapsToRectifyFisheyeImage(fisheye_camera_info_, &cv_warp_map_x_, &cv_warp_map_y_);
  fisheye_image_rect_.create(fisheye_camera_info_.height, fisheye_camera_info_.width, CV_8UC1);

  TangoService_getCameraIntrinsics(TANGO_CAMERA_COLOR, &tango_camera_intrinsics);
  toCameraInfo(tango_camera_intrinsics, &color_camera_info_);
  color_camera_info_manager_.reset(new camera_info_manager::CameraInfoManager(node_handle_));
  color_camera_info_manager_->setCameraName("color_1");
  // Cache camera model for more efficiency.
  color_camera_model_.fromCameraInfo(color_camera_info_);

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

  bool enable_drift_correction = false;
  int localization_mode;
  node_handle_.param(publisher_config_.localization_mode_param, localization_mode, LocalizationMode::ODOMETRY);
  if (localization_mode == LocalizationMode::ONLINE_SLAM) {
    enable_drift_correction = true;
  }
  const char* config_enable_drift_correction = "config_enable_drift_correction";
  result = TangoConfig_setBool(tango_config_, config_enable_drift_correction, enable_drift_correction);
  if(result != TANGO_SUCCESS) {
    LOG(ERROR) << function_name << ", TangoConfig_setBool "
        << config_enable_drift_correction << " error: " << result;
    return result;
  }
  const char* config_enable_auto_recovery = "config_enable_auto_recovery";
  result = TangoConfig_setBool(tango_config_, config_enable_auto_recovery, true);
  if(result != TANGO_SUCCESS) {
    LOG(ERROR) << function_name << ", TangoConfig_setBool "
        << config_enable_auto_recovery << " error: " << result;
    return result;
  }
  const char* config_enable_depth = "config_enable_depth";
  result = TangoConfig_setBool(tango_config_, config_enable_depth, true);
  if(result != TANGO_SUCCESS) {
    LOG(ERROR) << function_name << ", TangoConfig_setBool "
        << config_enable_depth << " error: " << result;
    return result;
  }
  const char* config_depth_mode = "config_depth_mode";
  result = TangoConfig_setInt32(tango_config_, config_depth_mode, TANGO_POINTCLOUD_XYZC);
  if(result != TANGO_SUCCESS) {
    LOG(ERROR) << function_name << ", TangoConfig_setInt "
        << config_depth_mode << " error: " << result;
    return result;
  }
  const char* config_enable_color_camera = "config_enable_color_camera";
  result = TangoConfig_setBool(tango_config_, config_enable_color_camera, true);
  if(result != TANGO_SUCCESS) {
    LOG(ERROR) << function_name << ", TangoConfig_setBool "
        << config_enable_color_camera << " error: " << result;
    return result;
  }
  return TANGO_SUCCESS;
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

void TangoRosNode::TangoDisconnect() {
  TangoConfig_free(tango_config_);
  tango_config_ = nullptr;
  TangoService_disconnect();
}

void TangoRosNode::PublishStaticTransforms() {
  TangoCoordinateFramePair pair;
  TangoPoseData pose;

  pair.base = TANGO_COORDINATE_FRAME_DEVICE;
  pair.target = TANGO_COORDINATE_FRAME_IMU;
  TangoService_getPoseAtTime(0.0, pair, &pose);
  geometry_msgs::TransformStamped device_T_imu;
  toTransformStamped(pose, time_offset_, &device_T_imu);
  device_T_imu.header.frame_id = toFrameId(TANGO_COORDINATE_FRAME_DEVICE);
  device_T_imu.child_frame_id = toFrameId(TANGO_COORDINATE_FRAME_IMU);
  device_T_imu.header.stamp = ros::Time::now();
  tf_static_broadcaster_.sendTransform(device_T_imu);

  if (publisher_config_.publish_point_cloud || publisher_config_.publish_laser_scan) {
    pair.base = TANGO_COORDINATE_FRAME_DEVICE;
    pair.target = TANGO_COORDINATE_FRAME_CAMERA_DEPTH;
    TangoService_getPoseAtTime(0.0, pair, &pose);
    toTransformStamped(pose, time_offset_, &device_T_camera_depth_);
    device_T_camera_depth_.header.frame_id = toFrameId(TANGO_COORDINATE_FRAME_DEVICE);
    device_T_camera_depth_.child_frame_id = toFrameId(TANGO_COORDINATE_FRAME_CAMERA_DEPTH);
    device_T_camera_depth_.header.stamp = ros::Time::now();
    tf_static_broadcaster_.sendTransform(device_T_camera_depth_);
  }

  if (publisher_config_.publish_laser_scan) {
    // According to the ROS documentation, laser scan angles are measured around
    // the Z-axis in the laser scan frame. To follow this convention the laser
    // scan frame has to be rotated of 90 degrees around x axis with respect to
    // the Tango point cloud frame.
    camera_depth_T_laser_ = tf::StampedTransform(
        tf::Transform(tf::Quaternion(1 / sqrt(2), 0, 0, 1 / sqrt(2))), ros::Time::now(),
                      toFrameId(TANGO_COORDINATE_FRAME_CAMERA_DEPTH), LASER_SCAN_FRAME_ID);
    geometry_msgs::TransformStamped camera_depth_T_laser_message;
    tf::transformStampedTFToMsg(camera_depth_T_laser_, camera_depth_T_laser_message);
    tf_static_broadcaster_.sendTransform(camera_depth_T_laser_message);
  }

  if (publisher_config_.publish_camera & CAMERA_FISHEYE) {
    pair.base = TANGO_COORDINATE_FRAME_DEVICE;
    pair.target = TANGO_COORDINATE_FRAME_CAMERA_FISHEYE;
    TangoService_getPoseAtTime(0.0, pair, &pose);
    toTransformStamped(pose, time_offset_, &device_T_camera_fisheye_);
    device_T_camera_fisheye_.header.frame_id = toFrameId(TANGO_COORDINATE_FRAME_DEVICE);
    device_T_camera_fisheye_.child_frame_id = toFrameId(TANGO_COORDINATE_FRAME_CAMERA_FISHEYE);
    device_T_camera_fisheye_.header.stamp = ros::Time::now();
    tf_static_broadcaster_.sendTransform(device_T_camera_fisheye_);
  }

  if (publisher_config_.publish_camera & CAMERA_COLOR) {
    pair.base = TANGO_COORDINATE_FRAME_DEVICE;
    pair.target = TANGO_COORDINATE_FRAME_CAMERA_COLOR;
    TangoService_getPoseAtTime(0.0, pair, &pose);
    toTransformStamped(pose, time_offset_, &device_T_camera_color_);
    device_T_camera_color_.header.frame_id = toFrameId(TANGO_COORDINATE_FRAME_DEVICE);
    device_T_camera_color_.child_frame_id = toFrameId(TANGO_COORDINATE_FRAME_CAMERA_COLOR);
    device_T_camera_color_.header.stamp = ros::Time::now();
    tf_static_broadcaster_.sendTransform(device_T_camera_color_);
  }
}

void TangoRosNode::OnPoseAvailable(const TangoPoseData* pose) {
  if (publisher_config_.publish_device_pose) {
    if (pose->frame.base == TANGO_COORDINATE_FRAME_START_OF_SERVICE
        && pose->frame.target == TANGO_COORDINATE_FRAME_DEVICE) {
      if (pose->status_code == TANGO_POSE_VALID && pose_available_mutex_.try_lock()) {
        toTransformStamped(*pose, time_offset_, &start_of_service_T_device_);
        start_of_service_T_device_.header.frame_id =
          toFrameId(TANGO_COORDINATE_FRAME_START_OF_SERVICE);
        start_of_service_T_device_.child_frame_id =
          toFrameId(TANGO_COORDINATE_FRAME_DEVICE);
        TangoCoordinateFramePair pair;
        pair.base = TANGO_COORDINATE_FRAME_AREA_DESCRIPTION;
        pair.target = TANGO_COORDINATE_FRAME_START_OF_SERVICE;
        TangoPoseData area_description_T_start_of_service;
        TangoService_getPoseAtTime(0.0, pair, &area_description_T_start_of_service);
        if (area_description_T_start_of_service.status_code == TANGO_POSE_VALID) {
          toTransformStamped(area_description_T_start_of_service,
                             time_offset_, &area_description_T_start_of_service_);
          area_description_T_start_of_service_.header.frame_id =
              toFrameId(TANGO_COORDINATE_FRAME_AREA_DESCRIPTION);
          area_description_T_start_of_service_.child_frame_id =
              toFrameId(TANGO_COORDINATE_FRAME_START_OF_SERVICE);
        }
        pose_available_.notify_all();
        pose_available_mutex_.unlock();
      }
    }
  }
}

void TangoRosNode::OnPointCloudAvailable(const TangoPointCloud* point_cloud) {
  if (point_cloud->num_points > 0) {
    if (publisher_config_.publish_point_cloud && point_cloud_available_mutex_.try_lock()) {
      toPointCloud2(*point_cloud, time_offset_, &point_cloud_);
      point_cloud_.header.frame_id = toFrameId(TANGO_COORDINATE_FRAME_CAMERA_DEPTH);
      point_cloud_available_.notify_all();
      point_cloud_available_mutex_.unlock();
    }
    if (publisher_config_.publish_laser_scan && laser_scan_available_mutex_.try_lock()) {
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
      toLaserScan(*point_cloud, time_offset_, laser_scan_min_height_,
                  laser_scan_max_height_, camera_depth_T_laser_, &laser_scan_);
      laser_scan_.header.frame_id = LASER_SCAN_FRAME_ID;
      laser_scan_available_.notify_all();
      laser_scan_available_mutex_.unlock();
    }
  }
}

void TangoRosNode::OnFrameAvailable(TangoCameraId camera_id, const TangoImageBuffer* buffer) {
  if ((publisher_config_.publish_camera & CAMERA_FISHEYE) &&
       camera_id == TangoCameraId::TANGO_CAMERA_FISHEYE &&
       fisheye_image_available_mutex_.try_lock()) {
    fisheye_image_ = cv::Mat(buffer->height + buffer->height / 2, buffer->width,
                             CV_8UC1, buffer->data, buffer->stride); // No deep copy.
    fisheye_image_header_.stamp.fromSec(buffer->timestamp + time_offset_);
    fisheye_image_header_.seq = buffer->frame_number;
    fisheye_image_header_.frame_id = toFrameId(TANGO_COORDINATE_FRAME_CAMERA_FISHEYE);
    fisheye_image_available_.notify_all();
    fisheye_image_available_mutex_.unlock();
  }
  if ((publisher_config_.publish_camera & CAMERA_COLOR) &&
       camera_id == TangoCameraId::TANGO_CAMERA_COLOR &&
       color_image_available_mutex_.try_lock()) {
    color_image_ = cv::Mat(buffer->height + buffer->height / 2, buffer->width,
                           CV_8UC1, buffer->data, buffer->stride); // No deep copy.
    color_image_header_.stamp.fromSec(buffer->timestamp + time_offset_);
    color_image_header_.seq = buffer->frame_number;
    color_image_header_.frame_id = toFrameId(TANGO_COORDINATE_FRAME_CAMERA_COLOR);
    color_image_available_.notify_all();
    color_image_available_mutex_.unlock();
  }
}

void TangoRosNode::StartPublishing() {
  run_threads_ = true;
  publish_device_pose_thread_ = std::thread(&TangoRosNode::PublishDevicePose, this);
  publish_pointcloud_thread_ = std::thread(&TangoRosNode::PublishPointCloud, this);
  publish_laserscan_thread_ = std::thread(&TangoRosNode::PublishLaserScan, this);
  publish_fisheye_image_thread_ = std::thread(&TangoRosNode::PublishFisheyeImage, this);
  publish_color_image_thread_ = std::thread(&TangoRosNode::PublishColorImage, this);
  ros_spin_thread_ = std::thread(&TangoRosNode::RunRosSpin, this);
}

void TangoRosNode::StopPublishing() {
  if (run_threads_) {
    run_threads_ = false;
    if (publisher_config_.publish_device_pose)
      publish_device_pose_thread_.join();
    if (publisher_config_.publish_point_cloud)
      publish_pointcloud_thread_.join();
    if (publisher_config_.publish_laser_scan)
      publish_laserscan_thread_.join();
    if (publisher_config_.publish_camera & CAMERA_FISHEYE)
      publish_fisheye_image_thread_.join();
    if (publisher_config_.publish_camera & CAMERA_COLOR)
      publish_color_image_thread_.join();
    ros_spin_thread_.join();
  }
}

void TangoRosNode::PublishDevicePose() {
  while(ros::ok()) {
    if (!run_threads_) {
      break;
    }
    {
      std::unique_lock<std::mutex> lock(pose_available_mutex_);
      pose_available_.wait(lock);
      if (publisher_config_.publish_device_pose) {
        tf_broadcaster_.sendTransform(start_of_service_T_device_);
        if (area_description_T_start_of_service_.child_frame_id != "") {
          // This transform can be empty. Don't publish it in this case.
          tf_broadcaster_.sendTransform(area_description_T_start_of_service_);
        }
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
      std::unique_lock<std::mutex> lock(point_cloud_available_mutex_);
      point_cloud_available_.wait(lock);
      if (publisher_config_.publish_point_cloud) {
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
      std::unique_lock<std::mutex> lock(laser_scan_available_mutex_);
      laser_scan_available_.wait(lock);
      if (publisher_config_.publish_laser_scan) {
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
      std::unique_lock<std::mutex> lock(fisheye_image_available_mutex_);
      fisheye_image_available_.wait(lock);
      if ((publisher_config_.publish_camera & CAMERA_FISHEYE)) {
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
      std::unique_lock<std::mutex> lock(color_image_available_mutex_);
      color_image_available_.wait(lock);
      if ((publisher_config_.publish_camera & CAMERA_COLOR)) {
        // The Tango image encoding is not supported by ROS.
        // We need to convert it to RGB.
        cv::Mat color_image_rgb;
        cv::cvtColor(color_image_, color_image_rgb, cv::COLOR_YUV420sp2BGRA);
        cv_bridge::CvImage cv_bridge_color_image;
        cv_bridge_color_image.header = color_image_header_;
        cv_bridge_color_image.encoding = sensor_msgs::image_encodings::BGRA8;
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

void TangoRosNode::DynamicReconfigureCallback(PublisherConfig &config, uint32_t level) {
  publisher_config_.publish_device_pose = config.publish_device_pose;
  publisher_config_.publish_point_cloud = config.publish_point_cloud;
  publisher_config_.publish_laser_scan = config.publish_laser_scan;
  if (config.publish_fisheye_camera) {
    publisher_config_.publish_camera |= CAMERA_FISHEYE;
  } else {
    publisher_config_.publish_camera &= ~CAMERA_FISHEYE;
  }
  if (config.publish_color_camera) {
    publisher_config_.publish_camera |= CAMERA_COLOR;
  } else {
    publisher_config_.publish_camera &= ~CAMERA_COLOR;
  }
  PublishStaticTransforms();
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
} // namespace tango_ros_native
