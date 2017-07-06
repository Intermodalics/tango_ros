// Copyright 2017 Intermodalics All Rights Reserved.
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

#include <glog/logging.h>

#include <ros/ros.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace tango_ros_conversions_helper {
void toTransformStamped(const TangoPoseData& pose,
                        double time_offset,
                        geometry_msgs::TransformStamped* transform) {
  transform->header.stamp.fromSec(pose.timestamp + time_offset);
  transform->header.frame_id = toFrameId(pose.frame.base);
  transform->child_frame_id = toFrameId(pose.frame.target);

  transform->transform.translation.x = pose.translation[0];
  transform->transform.translation.y = pose.translation[1];
  transform->transform.translation.z = pose.translation[2];
  transform->transform.rotation.x = pose.orientation[0];
  transform->transform.rotation.y = pose.orientation[1];
  transform->transform.rotation.z = pose.orientation[2];
  transform->transform.rotation.w = pose.orientation[3];
}

void toPointCloud2(const TangoPointCloud& tango_point_cloud,
                   double time_offset,
                   sensor_msgs::PointCloud2* point_cloud) {
  point_cloud->width = tango_point_cloud.num_points;
  point_cloud->height = 1;
  point_cloud->point_step = (sizeof(float) * NUMBER_OF_FIELDS_IN_POINT_CLOUD);
  point_cloud->is_dense = true;
  point_cloud->row_step = point_cloud->width;
  point_cloud->is_bigendian = false;
  point_cloud->data.resize(tango_point_cloud.num_points);
  sensor_msgs::PointCloud2Modifier modifier(*point_cloud);
  modifier.setPointCloud2Fields(NUMBER_OF_FIELDS_IN_POINT_CLOUD,
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
      ROS_WARN("Unknown TangoCoordinateFrameType: %d", tango_frame_type);
      string_frame_type = "unknown";
      break;
  }
  return string_frame_type;
}

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
    ROS_WARN("Unknown camera ID: %d", camera_intrinsics.camera_id);
  }
}

void toTango3DR_CameraCalibration(
    const TangoCameraIntrinsics& camera_intrinsics,
    Tango3DR_CameraCalibration* t3dr_camera_intrinsics) {
  t3dr_camera_intrinsics->calibration_type =
          static_cast<Tango3DR_TangoCalibrationType>(camera_intrinsics.calibration_type);
  t3dr_camera_intrinsics->width = camera_intrinsics.width;
  t3dr_camera_intrinsics->height = camera_intrinsics.height;
  t3dr_camera_intrinsics->fx = camera_intrinsics.fx;
  t3dr_camera_intrinsics->fy = camera_intrinsics.fy;
  t3dr_camera_intrinsics->cx = camera_intrinsics.cx;
  t3dr_camera_intrinsics->cy = camera_intrinsics.cy;
  std::copy(std::begin(camera_intrinsics.distortion),
            std::end(camera_intrinsics.distortion),
            std::begin(t3dr_camera_intrinsics->distortion));
}

void toTango3DR_Pose(const TangoPoseData& tango_pose_data, Tango3DR_Pose* t3dr_pose) {
  std::copy(std::begin(tango_pose_data.translation),
            std::end(tango_pose_data.translation),
            std::begin(t3dr_pose->translation));
  std::copy(std::begin(tango_pose_data.orientation),
            std::end(tango_pose_data.orientation),
            std::begin(t3dr_pose->orientation));
}

void toMeshMarker(const Tango3DR_GridIndex& grid_index,
                  const Tango3DR_Mesh& tango_mesh,
                  double time_offset,
                  visualization_msgs::Marker* mesh_marker) {
  mesh_marker->header.frame_id = toFrameId(TANGO_COORDINATE_FRAME_START_OF_SERVICE);
  mesh_marker->header.stamp.fromSec(tango_mesh.timestamp + time_offset);
  mesh_marker->ns = "tango";
  mesh_marker->type = visualization_msgs::Marker::TRIANGLE_LIST;
  mesh_marker->action = visualization_msgs::Marker::ADD;
  mesh_marker->pose.orientation.w = 1.0;
  mesh_marker->scale.x = 1.0;
  mesh_marker->scale.y = 1.0;
  mesh_marker->scale.z = 1.0;
  mesh_marker->color.r = 1.0;
  mesh_marker->color.g = 1.0;
  mesh_marker->color.b = 1.0;
  mesh_marker->color.a = 1.0;
  mesh_marker->lifetime = ros::Duration(0);
  // Make unique id from tango mesh indices.
  mesh_marker->id = grid_index[0];
  mesh_marker->id *= 37;
  mesh_marker->id += grid_index[1];
  mesh_marker->id *= 37;
  mesh_marker->id += grid_index[2];
  mesh_marker->points.resize(tango_mesh.num_faces * 3);
  mesh_marker->colors.resize(tango_mesh.num_faces * 3);
  for (size_t i = 0; i < tango_mesh.num_faces; ++i) {
    // Add the 3 points of the triangle face and the corresponding colors.
    geometry_msgs::Point point;
    toPoint(tango_mesh.vertices[tango_mesh.faces[i][0]], &point);
    mesh_marker->points[i * 3] = point;
    toPoint(tango_mesh.vertices[tango_mesh.faces[i][1]], &point);
    mesh_marker->points[i * 3 + 1] = point;
    toPoint(tango_mesh.vertices[tango_mesh.faces[i][2]], &point);
    mesh_marker->points[i * 3 + 2] = point;

    std_msgs::ColorRGBA color;
    toColor(tango_mesh.colors[tango_mesh.faces[i][0]], &color);
    mesh_marker->colors[i * 3] = color;
    toColor(tango_mesh.colors[tango_mesh.faces[i][1]], &color);
    mesh_marker->colors[i * 3 + 1] = color;
    toColor(tango_mesh.colors[tango_mesh.faces[i][2]], &color);
    mesh_marker->colors[i * 3 + 2] = color;
  }
}

void toOccupancyGrid(const Tango3DR_ImageBuffer& image_grid,
                     const Tango3DR_Vector2& origin,
                     double time_offset, double resolution, uint8_t threshold,
                     nav_msgs::OccupancyGrid* occupancy_grid) {
  occupancy_grid->header.frame_id = toFrameId(TANGO_COORDINATE_FRAME_START_OF_SERVICE);
  occupancy_grid->header.stamp.fromSec(image_grid.timestamp + time_offset);
  occupancy_grid->info.map_load_time = occupancy_grid->header.stamp;
  occupancy_grid->info.width = image_grid.width;
  occupancy_grid->info.height = image_grid.height;
  occupancy_grid->info.resolution = resolution;
  occupancy_grid->info.origin.position.x = origin[0];
  // We have the position of the top left pixel, instead we want the position
  // of the bottom left pixel.
  occupancy_grid->info.origin.position.y = origin[1] - image_grid.height * resolution;

  occupancy_grid->data.reserve(image_grid.height * image_grid.width);
  for (size_t i = 0; i < image_grid.height; ++i) {
    for (size_t j = 0; j < image_grid.width; ++j) {
      // The image uses a coordinate system with (x: right, y: down), while
      // the occupancy grid is using (x: right, y: up). The image is therefore
      // flipped around the x axis.
      uint8_t value = image_grid.data[j + (image_grid.height - i - 1) * image_grid.width];
      if (value == 128) {
        occupancy_grid->data.push_back(UNKNOWN_CELL);
      } else if (value <= threshold) {
        occupancy_grid->data.push_back(FREE_CELL);
      } else if (value > threshold) {
        occupancy_grid->data.push_back(OCCUPIED_CELL);
      } else {
        LOG(WARNING) << "Unknown value: " << static_cast<int>(value);
      }
    }
  }
}

void toPoint(const Tango3DR_Vector3& tango_vector, geometry_msgs::Point* point) {
  point->x = tango_vector[0];
  point->y = tango_vector[1];
  point->z = tango_vector[2];
}

void toColor(const Tango3DR_Color& tango_color, std_msgs::ColorRGBA* color) {
  color->r = tango_color[0] / 255.;
  color->g = tango_color[1] / 255.;
  color->b = tango_color[2] / 255.;
  color->a = tango_color[3] / 255.;
}
} // namespace tango_ros_conversions_helper
