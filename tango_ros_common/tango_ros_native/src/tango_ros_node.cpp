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

#include <Eigen/Geometry>
#include <glog/logging.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/server.h>
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
// @param time_offset, offset in ms between pose (tango time) and
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
  transform->header.stamp.fromSec((pose.timestamp + time_offset) / 1e3);
}
// Converts a TangoPointCloud to a sensor_msgs::PointCloud2.
// @param tango_point_cloud, TangoPointCloud to convert.
// @param time_offset, offset in ms between tango_point_cloud (tango time) and
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
  point_cloud->header.stamp.fromSec((tango_point_cloud.timestamp + time_offset) / 1e3);
}
// Compresses a cv::Mat image to a sensor_msgs::CompressedImage in JPEG format.
// @param image, cv::Mat to compress.
// @param compressing_quality, value from 0 to 100 (the higher is the better).
// @param compressed_image, the output CompressedImage.
void compressImage(const cv::Mat& image, const char* compressing_format,
                       int compressing_quality,
                       sensor_msgs::CompressedImage* compressed_image) {
  cv::Mat image_good_endcoding = cv::Mat();
  cv::cvtColor(image, image_good_endcoding, cv::COLOR_YUV420sp2RGBA);
  std::vector<int> params {CV_IMWRITE_JPEG_QUALITY, compressing_quality};
  cv::imencode(compressing_format, image_good_endcoding, compressed_image->data, params);
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
}  // namespace

namespace tango_ros_native {
TangoRosNode::TangoRosNode(bool publish_device_pose, bool publish_point_cloud,
                           uint32_t publish_camera) :
    run_threads_(false) {
  publisher_config_.publish_device_pose = publish_device_pose;
  publisher_config_.publish_point_cloud = publish_point_cloud;
  publisher_config_.publish_camera = publish_camera;

  const  uint32_t queue_size = 1;
  const bool latching = true;
  point_cloud_publisher_ =
      node_handle_.advertise<sensor_msgs::PointCloud2>(publisher_config_.point_cloud_topic,
      queue_size, latching);

  fisheye_image_publisher_ =
        node_handle_.advertise<sensor_msgs::CompressedImage>(publisher_config_.fisheye_camera_topic,
        queue_size, latching);

  color_image_publisher_ =
      node_handle_.advertise<sensor_msgs::CompressedImage>(publisher_config_.color_camera_topic,
      queue_size, latching);
}

TangoRosNode::~TangoRosNode() {
  StopPublishing();
  if (tango_config_ != nullptr) {
    TangoConfig_free(tango_config_);
  }
}

bool TangoRosNode::OnTangoServiceConnected() {
  if (TangoSetupConfig() != TANGO_SUCCESS) {
      LOG(ERROR) << "Error while setting up Tango config.";
      return false;
  }
  if (TangoConnect() != TANGO_SUCCESS) {
    LOG(ERROR) << "Error while connecting to Tango Service.";
    return false;
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
    return false;
  }
  time_offset_ =  ros::Time::now().toSec() * 1e3 - pose.timestamp;
  return true;
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
  const char* config_enable_drift_correction = "config_enable_drift_correction";
  result = TangoConfig_setBool(tango_config_, config_enable_drift_correction, true);
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

void TangoRosNode::UpdatePublisherConfiguration(bool publish_device_pose,
                                                bool publish_point_cloud,
                                                uint32_t publish_camera) {
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::Config config;
  dynamic_reconfigure::BoolParameter dynamic_boolean_param;
  dynamic_boolean_param.name = "publish_device_pose";
  dynamic_boolean_param.value = publish_device_pose;
  config.bools.push_back(dynamic_boolean_param);

  dynamic_boolean_param.name = "publish_point_cloud";
  dynamic_boolean_param.value = publish_point_cloud;
  config.bools.push_back(dynamic_boolean_param);

  dynamic_boolean_param.name = "publish_camera_fisheye";
  dynamic_boolean_param.value = publish_camera & CAMERA_FISHEYE;
  config.bools.push_back(dynamic_boolean_param);

  dynamic_boolean_param.name = "publish_camera_color";
  dynamic_boolean_param.value = publish_camera & CAMERA_COLOR;
  config.bools.push_back(dynamic_boolean_param);

  srv_req.config = config;
  if(!ros::service::call("/" + NODE_NAME + "/set_parameters", srv_req, srv_resp)) {
    LOG(ERROR) << "Service call failed, could not update dynamic reconfigure parameters.";
  }
}

void TangoRosNode::GetPublisherConfiguration(bool* publish_device_pose,
                               bool* publish_point_cloud,
                               uint32_t* publish_camera) {
  *publish_device_pose = publisher_config_.publish_device_pose;
  *publish_point_cloud = publisher_config_.publish_point_cloud;
  *publish_camera = publisher_config_.publish_camera;
}

void TangoRosNode::PublishStaticTransforms() {
  TangoCoordinateFramePair pair;
  TangoPoseData pose;
  if (publisher_config_.publish_point_cloud) {
    pair.base = TANGO_COORDINATE_FRAME_DEVICE;
    pair.target = TANGO_COORDINATE_FRAME_CAMERA_DEPTH;
    TangoService_getPoseAtTime(0.0, pair, &pose);
    toTransformStamped(pose, time_offset_, &device_T_camera_depth_);
    device_T_camera_depth_.header.frame_id = toFrameId(TANGO_COORDINATE_FRAME_DEVICE);
    device_T_camera_depth_.child_frame_id = toFrameId(TANGO_COORDINATE_FRAME_CAMERA_DEPTH);
    device_T_camera_depth_.header.stamp = ros::Time::now();
    tf_static_broadcaster_.sendTransform(device_T_camera_depth_);
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
        is_new_pose_available_ = true;
        pose_available_.notify_all();
        pose_available_mutex_.unlock();
      }
    }
  }
}

void TangoRosNode::OnPointCloudAvailable(const TangoPointCloud* point_cloud) {
  if (publisher_config_.publish_point_cloud && point_cloud->num_points > 0
      && point_cloud_available_mutex_.try_lock()) {
    toPointCloud2(*point_cloud, time_offset_, &point_cloud_);
    point_cloud_.header.frame_id = toFrameId(TANGO_COORDINATE_FRAME_CAMERA_DEPTH);
    is_new_point_cloud_available_ = true;
    point_cloud_available_.notify_all();
    point_cloud_available_mutex_.unlock();
  }
}

void TangoRosNode::OnFrameAvailable(TangoCameraId camera_id, const TangoImageBuffer* buffer) {
  if ((publisher_config_.publish_camera & CAMERA_FISHEYE) &&
       camera_id == TangoCameraId::TANGO_CAMERA_FISHEYE &&
       fisheye_image_available_mutex_.try_lock()) {
    fisheye_image_ = cv::Mat(buffer->height + buffer->height / 2, buffer->width,
                             CV_8UC1, buffer->data, buffer->stride); // No deep copy.
    fisheye_compressed_image_.header.stamp.fromSec((buffer->timestamp + time_offset_) / 1e3);
    fisheye_compressed_image_.header.seq = buffer->frame_number;
    fisheye_compressed_image_.header.frame_id = toFrameId(TANGO_COORDINATE_FRAME_CAMERA_FISHEYE);
    fisheye_compressed_image_.format = ROS_IMAGE_COMPRESSING_FORMAT;
    is_new_fisheye_image_available_ = true;
    fisheye_image_available_.notify_all();
    fisheye_image_available_mutex_.unlock();
  }
  if ((publisher_config_.publish_camera & CAMERA_COLOR) &&
       camera_id == TangoCameraId::TANGO_CAMERA_COLOR &&
       color_image_available_mutex_.try_lock()) {
    color_image_ = cv::Mat(buffer->height + buffer->height / 2, buffer->width,
                           CV_8UC1, buffer->data, buffer->stride); // No deep copy.
    color_compressed_image_.header.stamp.fromSec((buffer->timestamp + time_offset_) / 1e3);
    color_compressed_image_.header.seq = buffer->frame_number;
    color_compressed_image_.header.frame_id = toFrameId(TANGO_COORDINATE_FRAME_CAMERA_COLOR);
    color_compressed_image_.format = ROS_IMAGE_COMPRESSING_FORMAT;
    is_new_color_image_available_ = true;
    color_image_available_.notify_all();
    color_image_available_mutex_.unlock();
  }
}

void TangoRosNode::StartPublishing() {
  run_threads_ = true;
  publish_device_pose_thread_ = std::thread(&TangoRosNode::PublishDevicePose, this);
  publish_pointcloud_thread_ = std::thread(&TangoRosNode::PublishPointCloud, this);
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
      pose_available_.wait(lock, [this] { return is_new_pose_available_ == true; });
      if (publisher_config_.publish_device_pose) {
        tf_broadcaster_.sendTransform(start_of_service_T_device_);
      }
      is_new_pose_available_ = false;
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
      point_cloud_available_.wait(lock, [this] { return is_new_point_cloud_available_ == true; });
      if (publisher_config_.publish_point_cloud) {
        point_cloud_publisher_.publish(point_cloud_);
      }
      is_new_point_cloud_available_ = false;
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
      fisheye_image_available_.wait(lock, [this] { return is_new_fisheye_image_available_ == true; });
      if ((publisher_config_.publish_camera & CAMERA_FISHEYE)) {
        compressImage(fisheye_image_, CV_IMAGE_COMPRESSING_FORMAT,
                      IMAGE_COMPRESSING_QUALITY, &fisheye_compressed_image_);
        fisheye_image_publisher_.publish(fisheye_compressed_image_);
      }
      is_new_fisheye_image_available_ = false;
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
      color_image_available_.wait(lock, [this] { return is_new_color_image_available_ == true; });
      if ((publisher_config_.publish_camera & CAMERA_COLOR)) {
        compressImage(color_image_, CV_IMAGE_COMPRESSING_FORMAT,
                      IMAGE_COMPRESSING_QUALITY, &color_compressed_image_);
        color_image_publisher_.publish(color_compressed_image_);
      }
      is_new_color_image_available_ = false;
    }
  }
}

void TangoRosNode::DynamicReconfigureCallback(PublisherConfig &config, uint32_t level) {
  publisher_config_.publish_device_pose = config.publish_device_pose;
  publisher_config_.publish_point_cloud = config.publish_point_cloud;
  if (config.publish_camera_fisheye) {
    publisher_config_.publish_camera |= CAMERA_FISHEYE;
  } else {
    publisher_config_.publish_camera &= ~CAMERA_FISHEYE;
  }
  if (config.publish_camera_color) {
    publisher_config_.publish_camera |= CAMERA_COLOR;
  } else {
    publisher_config_.publish_camera &= ~CAMERA_COLOR;
  }
  PublishStaticTransforms();
}

void TangoRosNode::RunRosSpin() {
  dynamic_reconfigure::Server<tango_ros_native::PublisherConfig> server;
  dynamic_reconfigure::Server<tango_ros_native::PublisherConfig>::CallbackType callback;
  callback = boost::bind(&TangoRosNode::DynamicReconfigureCallback, this, _1, _2);
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
