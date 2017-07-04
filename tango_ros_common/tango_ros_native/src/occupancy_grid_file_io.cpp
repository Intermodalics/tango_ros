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
#include "tango_ros_native/occupancy_grid_file_io.h"

#include <cmath>
#include <fstream>
#include <iostream>

#include <geometry_msgs/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>

#include <glog/logging.h>

#include <yaml-cpp/yaml.h>

namespace {
void AddTrailingSlashToDirectoryPathIfNeeded(std::string& directory_path) {
  if (!directory_path.empty() && directory_path.back() != '/') {
    directory_path += "/";
  }
}
} // namespace

namespace occupancy_grid_file_io {
bool SaveOccupancyGridToFiles(
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

  std::string image_name = map_name;
  if (image_name.empty())
    image_name = "\"\"";
  std::string uuid = map_uuid;
  if (uuid.empty())
    uuid = "\"\"";

  std::string map_yaml_file_path = map_directory_with_trailing_slash + map_name + ".yaml";
  FILE* yaml_file = fopen(map_yaml_file_path.c_str(), "w");
  if (!yaml_file) {
    LOG(ERROR) << "Could no open file " << map_yaml_file_path;
    return false;
  }

  tf::Matrix3x3 mat(tf::Quaternion(map_metadata.origin.orientation.x,
                                   map_metadata.origin.orientation.y,
                                   map_metadata.origin.orientation.z,
                                   map_metadata.origin.orientation.w));
  double yaw, pitch, roll;
  mat.getEulerYPR(yaw, pitch, roll);
  if (std::isnan(yaw))
    yaw = 0.0;

  fprintf(yaml_file, "image: %s.pgm\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\n"
      "occupied_thresh: 0.65\nfree_thresh: 0.196\nuuid: %s\n\n",
      map_name.c_str(), map_metadata.resolution, map_metadata.origin.position.x,
      map_metadata.origin.position.y, yaw, uuid.c_str());

  fclose(yaml_file);
  return true;
}

bool LoadOccupancyGridFromFiles(
    const std::string&  map_name, const std::string& map_directory,
    nav_msgs::OccupancyGrid* occupancy_grid, std::string* map_uuid) {
  int negate;
  double occupied_threshold;
  double free_threshold;
  bool result = LoadOccupancyGridMetadataFromYamlFile(
      map_name, map_directory, &(occupancy_grid->info), &negate,
      &occupied_threshold, &free_threshold, map_uuid);
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
  std::ifstream pgm_file(map_pgm_file_path, std::ios::binary);
  if (pgm_file.fail()) {
    LOG(ERROR) << "Could no open file " << map_pgm_file_path;
    return false;
  }

  // First line contains file type.
  std::string file_type;
  getline(pgm_file, file_type);
  LOG(INFO) << file_type;
  if (file_type.compare("P5") != 0) {
    LOG(ERROR) << "Pgm file type error. Type is " << file_type
        << " while supported type is P5.";
    return false;
  }
  // Second line contains comment.
  std::string comment;
  getline(pgm_file, comment);
  LOG(INFO) << comment;
  // Third line contains size.
  std::string image_size;
  getline(pgm_file, image_size);
  std::stringstream size_stream(image_size);
  size_stream >> occupancy_grid->info.width >> occupancy_grid->info.height;
  LOG(INFO) << "Image size is " << occupancy_grid->info.width << "x" << occupancy_grid->info.height;
  // Fourth line contains max value.
  std::string max_value_string;
  getline(pgm_file, max_value_string);
  std::stringstream max_val_stream(max_value_string);
  int max_value = 0;
  max_val_stream >> max_value;
  // Following lines contain binary data.
  int pixel_array[occupancy_grid->info.height * occupancy_grid->info.width];
  for (size_t i = 0; i < occupancy_grid->info.height * occupancy_grid->info.width; ++i) {
    char pixel = pgm_file.get();
    pixel_array[i] = static_cast<int>(pixel);
  }
  // Need to invert the ordering of the pixels to
  // produce a map with cell (0,0) in the lower-left corner.
  occupancy_grid->data.reserve(occupancy_grid->info.height * occupancy_grid->info.width);
  for (size_t i = 0; i < occupancy_grid->info.height; ++i) {
    for (size_t j = 0; j < occupancy_grid->info.width; ++j) {
      int value = pixel_array[j + (occupancy_grid->info.height - i - 1) * occupancy_grid->info.width];
      if (negate)
        value = max_value - value;
      double occupancy = (max_value - value) / static_cast<double>(max_value);
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
    const std::string&  map_name, const std::string& map_directory,
    nav_msgs::MapMetaData* map_metadata, int* negate,
    double* occupied_threshold, double* free_threshold, std::string* map_uuid) {
  std::string map_directory_with_trailing_slash = map_directory;
  AddTrailingSlashToDirectoryPathIfNeeded(map_directory_with_trailing_slash);

  std::string map_yam_file_path = map_directory + map_name + ".yaml";
  std::ifstream yaml_file(map_yam_file_path.c_str());
  if (yaml_file.fail()) {
    LOG(ERROR) << "Could no open file " << map_yam_file_path;
    return false;
  }

  YAML::Node node = YAML::Load(yaml_file);
  try {
    map_metadata->resolution = node["resolution"].as<double>();
  } catch (YAML::RepresentationException& e) {
    LOG(ERROR) << "The map does not contain a resolution tag or it is invalid. " << e.msg;
    return false;
  }
  try {
    *negate = node["negate"].as<int>();
  } catch (YAML::RepresentationException& e) {
    LOG(ERROR) << "The map does not contain a negate tag or it is invalid. " << e.msg;
    return false;
  }
  try {
    *occupied_threshold = node["occupied_thresh"].as<double>();
  } catch (YAML::RepresentationException& e) {
    LOG(ERROR) << "The map does not contain an occupied_thresh tag or it is invalid. " << e.msg;
    return false;
  }
  try {
    *free_threshold = node["free_thresh"].as<double>();
  } catch (YAML::RepresentationException& e) {
    LOG(ERROR) << "The map does not contain a free_thresh tag or it is invalid. " << e.msg;
    return false;
  }
  try {
    map_metadata->origin.position.x = node["origin"][0].as<double>();
    map_metadata->origin.position.y = node["origin"][1].as<double>();
    map_metadata->origin.position.z = 0.0;
    tf::Quaternion quaternion;
    double yaw = node["origin"][2].as<double>();
    quaternion.setRPY(0., 0., yaw);
    map_metadata->origin.orientation.x = quaternion.x();
    map_metadata->origin.orientation.y = quaternion.y();
    map_metadata->origin.orientation.z = quaternion.z();
    map_metadata->origin.orientation.w = quaternion.w();
  } catch (YAML::RepresentationException& e) {
    LOG(ERROR) << "The map does not contain an origin tag or it is invalid. " << e.msg;
    return false;
  }
  try {
    *map_uuid = node["uuid"].as<std::string>();
  } catch (YAML::RepresentationException& e) {
    LOG(WARNING) << "The map does not contain a uuid tag or it is invalid. " << e.msg;
  }
  yaml_file.close();
  return true;
}
} // namespace occupancy_grid_file_io
