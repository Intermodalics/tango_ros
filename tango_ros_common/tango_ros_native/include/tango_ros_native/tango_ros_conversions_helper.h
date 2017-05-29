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
#ifndef TANGO_ROS_CONVERSIONS_HELPER_H_
#define TANGO_ROS_CONVERSIONS_HELPER_H_
#include <tango_3d_reconstruction/tango_3d_reconstruction_api.h>
#include <tango_client_api/tango_client_api.h>
#include <tango_support_api/tango_support_api.h>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <tf/LinearMath/Transform.h>
#include <visualization_msgs/MarkerArray.h>

namespace tango_ros_conversions_helper {
const int NUMBER_OF_FIELDS_IN_POINT_CLOUD = 4;
const float OCCUPANCY_GRID_RESOLUTION = 0.05; // in meter.
constexpr int FREE_CELL = 0;
constexpr int OCCUPIED_CELL = 100;
constexpr int UNKNOWN_CELL = -1;

// Converts a TangoPoseData to a geometry_msgs::TransformStamped.
// @param pose, TangoPoseData to convert.
// @param time_offset, offset in s between pose (tango time) and
//        transform (ros time).
// @param transform, the output TransformStamped.
void toTransformStamped(const TangoPoseData& pose,
                        double time_offset,
                        geometry_msgs::TransformStamped* transform);

// Converts a TangoPointCloud to a sensor_msgs::PointCloud2.
// @param tango_point_cloud, TangoPointCloud to convert.
// @param time_offset, offset in s between tango_point_cloud (tango time) and
//        point_cloud (ros time).
// @param point_cloud, the output PointCloud2.
void toPointCloud2(const TangoPointCloud& tango_point_cloud,
                   double time_offset,
                   sensor_msgs::PointCloud2* point_cloud);

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
                      double max_height, sensor_msgs::LaserScan* laser_scan);

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
                 sensor_msgs::LaserScan* laser_scan);

// Converts a TangoCoordinateFrameType to a ros frame ID i.e. a string.
// @param tango_frame_type, TangoCoordinateFrameType to convert.
// @return returns the corresponding frame id.
std::string toFrameId(const TangoCoordinateFrameType& tango_frame_type);

// Converts TangoCameraIntrinsics to sensor_msgs::CameraInfo.
// See Tango documentation:
// http://developers.google.com/tango/apis/unity/reference/class/tango/tango-camera-intrinsics
// And ROS documentation:
// http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
// @param camera_intrinsics, TangoCameraIntrinsics to convert.
// @param camera_info, the output CameraInfo.
void toCameraInfo(const TangoCameraIntrinsics& camera_intrinsics,
                  sensor_msgs::CameraInfo* camera_info);

// Converts TangoCameraIntrinsics to Tango3DR_CameraCalibration.
// @param camera_intrinsics, TangoCameraIntrinsics to convert.
// @param t3dr_camera_intrinsics, the output Tango3DR_CameraCalibration.
void toTango3DR_CameraCalibration(
    const TangoCameraIntrinsics& camera_intrinsics,
    Tango3DR_CameraCalibration* t3dr_camera_intrinsics);

// Converts TangoPoseData to Tango3DR_Pose.
// @param tango_pose_data, TangoPoseData to convert.
// @param t3dr_pose, the output Tango3DR_Pose.
void toTango3DR_Pose(const TangoPoseData& tango_pose_data, Tango3DR_Pose* t3dr_pose);

// Converts Tango3DR_Mesh to visualization_msgs::Marker (TRIANGLE_LIST).
// See ROS documentation:
// http://docs.ros.org/api/visualization_msgs/html/msg/Marker.html
// @param grid_index, index of the tango_mesh.
// @param tango_mesh, Tango3DR_Mesh to convert.
// @param mesh_marker, the output visualization_msgs::Marker.
void toMeshMarker(const Tango3DR_GridIndex& grid_index,
                  const Tango3DR_Mesh& tango_mesh,
                  double time_offset,
                  visualization_msgs::Marker* mesh_marker);

// Converts Tango3DR_ImageBuffer to nav_msgs::OccupancyGrid.
// See ROS documentation:
// http://docs.ros.org/kinetic/api/nav_msgs/html/msg/OccupancyGrid.html
// @param image_grid, Tango3DR_ImageBuffer to convert.
// @param origin, position of the top left pixel in
/// world coordinates (x: right, y: up).
// @param occupancy_grid, the output nav_msgs::OccupancyGrid.
void toOccupancyGrid(const Tango3DR_ImageBuffer& image_grid,
                     const Tango3DR_Vector2& origin,
                     double time_offset,
                     nav_msgs::OccupancyGrid* occupancy_grid);

// Converts Tango3DR_Vector3 to geometry_msgs::Point.
// @param tango_vector, Tango3DR_Vector3 to convert.
// @param point, the output geometry_msgs::Point.
void toPoint(const Tango3DR_Vector3& tango_vector, geometry_msgs::Point* point);

// Converts Tango3DR_Color to std_msgs::ColorRGBA.
// @param tango_color, Tango3DR_Color to convert.
// @param color, the output std_msgs::ColorRGBA.
void toColor(const Tango3DR_Color& tango_color, std_msgs::ColorRGBA* color);
} // namespace tango_ros_conversions_helper
#endif  // TANGO_ROS_CONVERSIONS_HELPER_H_
