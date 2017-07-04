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
#ifndef TANGO_3D_RECONSTRUCTION_HELPER_H_
#define TANGO_3D_RECONSTRUCTION_HELPER_H_
#include <string>

#include <tango_3d_reconstruction/tango_3d_reconstruction_api.h>
#include <tango_support_api/tango_support_api.h>

#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

namespace tango_3d_reconstruction_helper {
const double TANGO_3DR_DEFAULT_RESOLUTION = 0.05; // meter
const bool TANGO_3DR_DEFAULT_USE_SPACE_CLEARING = false;
const int32_t TANGO_3DR_DEFAULT_MIN_NUM_VERTICES = 1; // Default value from TangoConfig
const int32_t TANGO_3DR_DEFAULT_UPDATE_METHOD = 0; // TRAVERSAL_UPDATE
const int32_t TANGO_3DR_DEFAULT_MAX_VOXEL_WEIGHT = 16383; // Default value from TangoConfig
const double TANGO_3DR_DEFAULT_FLOORPLAN_MAX_ERROR = 0.; // meter

const std::string TANGO_3DR_RESOLUTION_PARAM_NAME = "reconstruction/resolution_3d";
const std::string TANGO_3DR_USE_SPACE_CLEARING_PARAM_NAME = "reconstruction/use_space_clearing";
const std::string TANGO_3DR_MIN_NUM_VERTICES_PARAM_NAME = "reconstruction/min_num_vertices";
const std::string TANGO_3DR_UPDATE_METHOD_PARAM_NAME = "reconstruction/update_method";
const std::string TANGO_3DR_MAX_VOXEL_WEIGHT_PARAM_NAME = "reconstruction/max_voxel_weight";
const std::string TANGO_3DR_FLOORPLAN_MAX_ERROR_PARAM_NAME = "reconstruction/floorplan_max_error";

Tango3DR_Status TangoSetup3DRConfig(
    const ros::NodeHandle& node_handle, double* t3dr_resolution,
    Tango3DR_ReconstructionContext* t3dr_context,
    Tango3DR_CameraCalibration* t3dr_color_camera_intrinsics);

void UpdateMesh(const Tango3DR_ReconstructionContext& t3dr_context,
         TangoSupportPointCloudManager* point_cloud_manager,
         TangoSupportImageBufferManager* image_buffer_manager,
         Tango3DR_Pose* last_camera_depth_pose,
         Tango3DR_Pose* last_camera_color_pose,
         Tango3DR_GridIndexArray* t3dr_updated_indices);

void ExtractMeshAndConvertToMarkerArray(
    const Tango3DR_ReconstructionContext& t3dr_context,
    const Tango3DR_GridIndexArray& t3dr_updated_indices,
    double time_offset, visualization_msgs::MarkerArray* mesh_marker_array);

bool ExtractFloorPlanImageAndConvertToOccupancyGrid(
    const Tango3DR_ReconstructionContext& t3dr_context,
    double time_offset, double t3dr_resolution,
    nav_msgs::OccupancyGrid* occupancy_grid);

} // namespace tango_3d_reconstruction_helper
#endif  // TANGO_3D_RECONSTRUCTION_HELPER_H_
