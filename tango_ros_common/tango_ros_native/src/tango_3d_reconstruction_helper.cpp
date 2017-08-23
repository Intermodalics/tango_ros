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
#include "tango_ros_native/tango_3d_reconstruction_helper.h"
#include "tango_ros_native/tango_ros_conversions_helper.h"

#include <glog/logging.h>

namespace tango_3d_reconstruction_helper {
Tango3DR_Status TangoSetup3DRConfig(
    const ros::NodeHandle& node_handle, double* t3dr_resolution,
    Tango3DR_ReconstructionContext* t3dr_context,
    Tango3DR_CameraCalibration* t3dr_color_camera_intrinsics) {
  const char* function_name = "TangoRosNodelet::TangoSetup3DRConfig()";

  Tango3DR_Config t3dr_config =
      Tango3DR_Config_create(TANGO_3DR_CONFIG_RECONSTRUCTION);
  Tango3DR_Status result;
  const char* resolution = "resolution";
  double t3dr_resolution_param;
  node_handle.param(TANGO_3DR_RESOLUTION_PARAM_NAME,
                    t3dr_resolution_param, TANGO_3DR_DEFAULT_RESOLUTION);
  *t3dr_resolution = t3dr_resolution_param;
  result = Tango3DR_Config_setDouble(t3dr_config, resolution, t3dr_resolution_param);
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
  result = Tango3DR_Config_setBool(t3dr_config, use_floorplan, true);
  if (result != TANGO_3DR_SUCCESS) {
    LOG(ERROR) << function_name << ", Tango3DR_Config_setBool "
        << use_floorplan << " error: " << result;
    return result;
  }
  const char* use_space_clearing = "use_space_clearing";
  bool t3dr_use_space_clearing;
  node_handle.param(TANGO_3DR_USE_SPACE_CLEARING_PARAM_NAME,
                     t3dr_use_space_clearing, TANGO_3DR_DEFAULT_USE_SPACE_CLEARING);
  result = Tango3DR_Config_setBool(t3dr_config, use_space_clearing,
                                   t3dr_use_space_clearing);
  if (result != TANGO_3DR_SUCCESS) {
    LOG(ERROR) << function_name << ", Tango3DR_Config_setBool "
        << use_space_clearing << " error: " << result;
    return result;
  }
  const char* min_num_vertices = "min_num_vertices";
  int t3dr_min_num_vertices;
  node_handle.param(TANGO_3DR_MIN_NUM_VERTICES_PARAM_NAME,
                     t3dr_min_num_vertices, TANGO_3DR_DEFAULT_MIN_NUM_VERTICES);
  result = Tango3DR_Config_setInt32(t3dr_config, min_num_vertices,
                                    t3dr_min_num_vertices);
  if (result != TANGO_3DR_SUCCESS) {
    LOG(ERROR) << function_name << ", Tango3DR_Config_setInt32 "
        << min_num_vertices << " error: " << result;
    return result;
  }
  const char* update_method = "update_method";
  int t3dr_update_method;
  node_handle.param(TANGO_3DR_UPDATE_METHOD_PARAM_NAME,
                     t3dr_update_method, TANGO_3DR_DEFAULT_UPDATE_METHOD);
  result = Tango3DR_Config_setInt32(t3dr_config, update_method,
                                    t3dr_update_method);
  if (result != TANGO_3DR_SUCCESS) {
    LOG(ERROR) << function_name << ", Tango3DR_Config_setInt32 "
        << update_method << " error: " << result;
    return result;
  }
  const char* max_voxel_weight = "max_voxel_weight";
  int t3dr_max_voxel_weight;
  node_handle.param(TANGO_3DR_MAX_VOXEL_WEIGHT_PARAM_NAME,
                     t3dr_max_voxel_weight, TANGO_3DR_DEFAULT_MAX_VOXEL_WEIGHT);
  result = Tango3DR_Config_setInt32(t3dr_config, max_voxel_weight,
                                    t3dr_max_voxel_weight);
  if (result != TANGO_3DR_SUCCESS) {
    LOG(ERROR) << function_name << ", Tango3DR_Config_setInt32 "
        << max_voxel_weight << " error: " << result;
    return result;
  }
  const char* floorplan_max_error = "floorplan_max_error";
  double t3dr_floorplan_max_error;
  node_handle.param(TANGO_3DR_FLOORPLAN_MAX_ERROR_PARAM_NAME,
                     t3dr_floorplan_max_error, TANGO_3DR_DEFAULT_FLOORPLAN_MAX_ERROR);
  result = Tango3DR_Config_setDouble(t3dr_config, floorplan_max_error,
                                     t3dr_floorplan_max_error);
  if (result != TANGO_3DR_SUCCESS) {
    LOG(ERROR) << function_name << ", Tango3DR_Config_setDouble "
        << floorplan_max_error << " error: " << result;
    return result;
  }

  *t3dr_context = Tango3DR_ReconstructionContext_create(t3dr_config);
  if (*t3dr_context == nullptr) {
    LOG(ERROR) << function_name << ", Tango3DR_ReconstructionContext_create error: "
        "Unable to create 3DR context.";
    return TANGO_3DR_ERROR;
  }
  // Configure the color camera intrinsics to be used with updates to the mesh.
  result = Tango3DR_ReconstructionContext_setColorCalibration(
      *t3dr_context, t3dr_color_camera_intrinsics);
  if (result != TANGO_3DR_SUCCESS) {
    LOG(ERROR) << function_name << ", Unable to set color calibration.";
    return TANGO_3DR_ERROR;
  }
  return Tango3DR_Config_destroy(t3dr_config);
}

void UpdateMesh(const Tango3DR_ReconstructionContext& t3dr_context,
                TangoSupport_PointCloudManager* point_cloud_manager,
                TangoSupport_ImageBufferManager* image_buffer_manager,
                Tango3DR_Pose* last_camera_depth_pose,
                Tango3DR_Pose* last_camera_color_pose,
                Tango3DR_GridIndexArray* t3dr_updated_indices)  {
  // Get latest point cloud.
  TangoPointCloud* last_point_cloud;
  TangoSupport_getLatestPointCloud(point_cloud_manager, &last_point_cloud);
  Tango3DR_PointCloud t3dr_depth;
  t3dr_depth.timestamp = last_point_cloud->timestamp;
  t3dr_depth.num_points = last_point_cloud->num_points;
  t3dr_depth.points = reinterpret_cast<Tango3DR_Vector4*>(
      last_point_cloud->points);
  // Get latest image.
  TangoImageBuffer* last_color_image_buffer;
  if (image_buffer_manager != nullptr) {
    TangoSupport_getLatestImageBuffer(image_buffer_manager,
                                      &last_color_image_buffer);
    Tango3DR_ImageBuffer t3dr_image;
    t3dr_image.width = last_color_image_buffer->width;
    t3dr_image.height = last_color_image_buffer->height;
    t3dr_image.stride = last_color_image_buffer->stride;
    t3dr_image.timestamp = last_color_image_buffer->timestamp;
    t3dr_image.format = static_cast<Tango3DR_ImageFormatType>(
        last_color_image_buffer->format);
    t3dr_image.data = last_color_image_buffer->data;
    // Get updated mesh segment indices.
    Tango3DR_Status result =
        Tango3DR_updateFromPointCloud(t3dr_context, &t3dr_depth, last_camera_depth_pose,
                        &t3dr_image, last_camera_color_pose,
                        t3dr_updated_indices);
    if (result != TANGO_3DR_SUCCESS) {
      LOG(ERROR) << "Tango3DR_updateFromPointCloud failed with error code "
          << result;
    }
  }
}

void ExtractMeshAndConvertToMarkerArray(
    const Tango3DR_ReconstructionContext& t3dr_context,
    const Tango3DR_GridIndexArray& t3dr_updated_indices,
    double time_offset, const std::string& base_frame_id,
    visualization_msgs::MarkerArray* mesh_marker_array) {
  for (size_t i = 0; i < t3dr_updated_indices.num_indices; ++i) {
    // Extract Tango mesh from updated index.
    Tango3DR_Mesh tango_mesh;
    Tango3DR_Status result = Tango3DR_extractMeshSegment(
        t3dr_context, t3dr_updated_indices.indices[i], &tango_mesh);
    if(result != TANGO_3DR_SUCCESS) {
      LOG(ERROR) << "Tango3DR_extractMeshSegment failed.";
      continue;
    }
    if (tango_mesh.num_faces == 0) {
      LOG(INFO) << "Empty mesh extracted.";
      continue;
    }
    // Make mesh marker from tango mesh.
    visualization_msgs::Marker mesh_marker;
    tango_ros_conversions_helper::toMeshMarker(t3dr_updated_indices.indices[i],
        tango_mesh, time_offset, base_frame_id, &mesh_marker);
    // Free tango mesh once we are finished with it.
    result = Tango3DR_Mesh_destroy(&tango_mesh);
    if (result != TANGO_3DR_SUCCESS) {
      LOG(ERROR) << "Tango3DR_Mesh_destroy failed with error code: "
          << result;
    }
    if (mesh_marker.points.empty()) {
      LOG(INFO) << "Empty mesh marker.";
      continue;
    }
    mesh_marker_array->markers.push_back(mesh_marker);
  }
}

bool ExtractFloorPlanImageAndConvertToOccupancyGrid(
    const Tango3DR_ReconstructionContext& t3dr_context,
    double time_offset, const std::string& base_frame_id,
    double t3dr_resolution, uint8_t threshold,
    nav_msgs::OccupancyGrid* occupancy_grid) {
  Tango3DR_Status result  = Tango3DR_updateFullFloorplan(t3dr_context);
  if (result == TANGO_3DR_SUCCESS) {
    Tango3DR_Vector2 origin;
    Tango3DR_ImageBuffer image_grid;
    result = Tango3DR_extractFullFloorplanImage(
        t3dr_context, TANGO_3DR_LAYER_SPACE, &origin, &image_grid);
    if (result == TANGO_3DR_SUCCESS) {
      tango_ros_conversions_helper::toOccupancyGrid(image_grid, origin,
          time_offset, base_frame_id, t3dr_resolution, threshold, occupancy_grid);
    } else {
      LOG(ERROR) << "Tango3DR_extractFullFloorplanImage failed with error"
          "code: " << result;
      return false;
    }
    result = Tango3DR_ImageBuffer_destroy(&image_grid);
    if (result != TANGO_3DR_SUCCESS) {
      LOG(ERROR) << "Tango3DR_ImageBuffer_destroy failed with error "
          "code: " << result;
      return false;
    }
  } else {
    LOG(ERROR) << "Tango3DR_updateFullFloorplan failed with error "
        "code: " << result;
    return false;
  }
  return true;
}
} // namespace tango_3d_reconstruction_helper
