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
#include "tango_ros_native/navigation_map_file_io.h"

#include <fstream>
#include <iostream>
#include <stdio.h>

#include <geometry_msgs/Quaternion.h>
#include "tf/LinearMath/Matrix3x3.h"

#include <glog/logging.h>

#include <yaml-cpp/yaml.h>

namespace {
void AddTrailingSlashToDirectoryPathIfNeeded(std::string& directory_path) {
  if (!directory_path.empty() && directory_path.back() != '/') {
    directory_path += "/";
  }
}
} // namespace

namespace navigation_map_file_io {
bool SaveOccupancyGridToNavigationMap(
    const std::string& map_name, const std::string& map_uuid,
    const std::string& map_directory, const nav_msgs::OccupancyGrid& occupancy_grid) {
  return SaveOccupancyGridDataToPgmFile(map_name, map_directory, occupancy_grid)
      && SaveOccupancyGridMetadataToYamlFile(map_name, map_uuid, map_directory, occupancy_grid.info);
}

bool SaveOccupancyGridDataToPgmFile(
    const std::string& map_name, const std::string& map_directory,
    const nav_msgs::OccupancyGrid& occupancy_grid) {
  std::string map_directory_with_trailing_slash = map_directory;
  AddTrailingSlashToDirectoryPathIfNeeded(map_directory_with_trailing_slash);

  std::string map_pgm_file_path = map_directory_with_trailing_slash + map_name + ".pgm";
  LOG(INFO) << "Writing occupancy grid data to " << map_pgm_file_path;
  FILE* pgm_file = fopen(map_pgm_file_path.c_str(), "w");
  if (!pgm_file) {
    LOG(ERROR) << "Could no open file " << map_pgm_file_path;
    return false;
  }
  fprintf(pgm_file, "P5\n# CREATOR: TangoRosStreamer %.3f m/pix\n%d %d\n255\n",
          occupancy_grid.info.resolution,
          occupancy_grid.info.width, occupancy_grid.info.height);
  for (size_t i = 0; i < occupancy_grid.info.height; ++i) {
    for (size_t j = 0; j < occupancy_grid.info.width; ++j) {
      // Need to invert the ordering of the cells to
      // produce an image with pixel (0,0) in the top-left corner.
      int value = occupancy_grid.data[j + (occupancy_grid.info.height - i - 1) * occupancy_grid.info.width];
      if (value == 0) {
        fputc(254, pgm_file);
      } else if (value == +100) {
        fputc(000, pgm_file);
      } else {
        fputc(205, pgm_file);
      }
    }
  }
  fclose(pgm_file);
  return true;
}

bool SaveOccupancyGridMetadataToYamlFile(
    const std::string& map_name, const std::string& map_uuid,
    const std::string& map_directory, const nav_msgs::MapMetaData& map_metadata) {
  std::string map_directory_with_trailing_slash = map_directory;
  AddTrailingSlashToDirectoryPathIfNeeded(map_directory_with_trailing_slash);

  std::string map_yaml_file_path = map_directory_with_trailing_slash + map_name + ".yaml";
  LOG(INFO) << "Writing occupancy grid metadata to " << map_yaml_file_path;
  FILE* yaml_file = fopen(map_yaml_file_path.c_str(), "w");
  if (!yaml_file) {
    LOG(ERROR) << "Could no open file " << map_yaml_file_path;
    return false;
  }

  geometry_msgs::Quaternion orientation = map_metadata.origin.orientation;
  tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
  double yaw, pitch, roll;
  mat.getEulerYPR(yaw, pitch, roll);

  fprintf(yaml_file, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\n"
      "occupied_thresh: 0.65\nfree_thresh: 0.196\nuuid: %s\n\n",
      map_name.c_str(), map_metadata.resolution, map_metadata.origin.position.x,
      map_metadata.origin.position.y, yaw, map_uuid.c_str());
  fclose(yaml_file);
  return true;
}

bool LoadOccupancyGridFromNavigationMap(
    const std::string&  map_name, const std::string& map_uuid,
    const std::string& map_directory, nav_msgs::OccupancyGrid* occupancy_grid) {
  int negate;
  double occupied_threshold;
  double free_threshold;
  bool result = LoadOccupancyGridMetadataFromYamlFile(
      map_name, map_uuid, map_directory, &(occupancy_grid->info), &negate,
      &occupied_threshold, &free_threshold);
  return result && LoadOccupancyGridDataFromPgmFile(
      map_name, map_directory, negate, occupied_threshold, free_threshold,
      occupancy_grid);
}

bool LoadOccupancyGridDataFromPgmFile(
    const std::string&  map_name, const std::string& map_directory,
    bool negate, double occupied_threshold, double free_threshold,
    nav_msgs::OccupancyGrid* occupancy_grid) {
  std::string map_directory_with_trailing_slash = map_directory;
  AddTrailingSlashToDirectoryPathIfNeeded(map_directory_with_trailing_slash);

  std::string map_pgm_file_path = map_directory_with_trailing_slash + map_name + ".pgm";
  std::ifstream pgm_file(map_pgm_file_path);
  if (pgm_file.fail()) {
    LOG(ERROR) << "Could no open file " << map_pgm_file_path;
    return false;
  }
  std::string inputLine = "";
  // First line contains the file type.
  getline(pgm_file, inputLine);
  if (inputLine.compare("P5") != 0) {
    LOG(ERROR) << "Pgm file type error. Type is " << inputLine
        << " while supported type is P5.";
    return false;
  }
  // Second line contains some comments.
  getline(pgm_file, inputLine);
  LOG(INFO) << "Comment : " << inputLine;
  // Third line contains the image size.
  int height = 0;
  int width = 0;
  std::stringstream ss;
  ss << pgm_file.rdbuf();
  ss >> width >> height;
  occupancy_grid->info.width = width;
  occupancy_grid->info.height = height;
  LOG(INFO) << "Image size is " << occupancy_grid->info.width << "x" << occupancy_grid->info.height;
  // Following lines contain data.
  int pixel_array[occupancy_grid->info.height * occupancy_grid->info.width];
  for (size_t i = 0; i < occupancy_grid->info.height * occupancy_grid->info.width; ++i) {
      ss >> pixel_array[i];
  }
  // Need to invert the ordering of the pixels to
  // produce a map with cell (0,0) in the lower-left corner.
  occupancy_grid->data.reserve(occupancy_grid->info.height * occupancy_grid->info.width);
  for (size_t i = 0; i < occupancy_grid->info.height; ++i) {
    for (size_t j = 0; j < occupancy_grid->info.width; ++j) {
      int value = pixel_array[j + (occupancy_grid->info.height - i - 1) * occupancy_grid->info.width];
      if (negate)
        value = 255 - value;
      double occupancy = (255 - value) / 255.0;
      if (occupancy < free_threshold) {
        occupancy_grid->data.push_back(0);
      } else if (occupancy > occupied_threshold) {
        occupancy_grid->data.push_back(100);
      } else {
        occupancy_grid->data.push_back(-1);
      }
    }
  }
  pgm_file.close();
  return true;
}

bool LoadOccupancyGridMetadataFromYamlFile(
    const std::string&  map_name, const std::string& map_uuid,
    const std::string& map_directory, nav_msgs::MapMetaData* map_metadata,
    int* negate, double* occupied_threshold, double* free_threshold) {
  std::string map_directory_with_trailing_slash = map_directory;
  AddTrailingSlashToDirectoryPathIfNeeded(map_directory_with_trailing_slash);

  std::string map_yam_file_path = map_directory + map_name + ".yaml";
  std::ifstream yaml(map_yam_file_path.c_str());
  if (yaml.fail()) {
    LOG(ERROR) << "Could no open file " << map_yam_file_path;
    return false;
  }

  YAML::Node doc = YAML::Load(yaml);
  try {
    map_metadata->resolution = doc["resolution"].as<double>();
  } catch (YAML::InvalidScalar) {
    LOG(ERROR) << "The map does not contain a resolution tag or it is invalid.";
    return false;
  }
  try {
    *negate = doc["negate"].as<int>();
  } catch (YAML::InvalidScalar) {
    LOG(ERROR) << "The map does not contain a negate tag or it is invalid.";
    return false;
  }
  try {
    *occupied_threshold = doc["occupied_thresh"].as<double>();
  } catch (YAML::InvalidScalar) {
    LOG(ERROR) << "The map does not contain an occupied_thresh tag or it is invalid.";
    return false;
  }
  try {
    *free_threshold = doc["free_thresh"].as<double>();
  } catch (YAML::InvalidScalar) {
    LOG(ERROR) << "The map does not contain a free_thresh tag or it is invalid.";
    return false;
  }
  try {
    map_metadata->origin.position.x = doc["origin"][0].as<double>();
    map_metadata->origin.position.y = doc["origin"][1].as<double>();
    map_metadata->origin.position.z = 0.0;
    tf::Quaternion quaternion;
    double yaw = doc["origin"][2].as<double>();
    quaternion.setRPY(0,0, yaw);
    map_metadata->origin.orientation.x = quaternion.x();
    map_metadata->origin.orientation.y = quaternion.y();
    map_metadata->origin.orientation.z = quaternion.z();
    map_metadata->origin.orientation.w = quaternion.w();
  } catch (YAML::InvalidScalar) {
    LOG(ERROR) << "The map does not contain an origin tag or it is invalid.";
    return false;
  }
  try {
    std::string uuid = doc["uuid"].as<std::string>();
    if (uuid.compare(map_uuid) != 0) {
      LOG(WARNING) << "The navigation map does not correspond to the localization map uuid."
          "Maps will not be aligned";
    }
  } catch (YAML::InvalidScalar) {
    LOG(WARNING) << "The map does not contain a uuid tag or it is invalid. "
    "The map will not be aligned.";
  }
  return true;
}
} // namespace navigation_map_file_io
