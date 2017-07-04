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
#ifndef OCCUPANCY_GRID_FILE_IO_H_
#define OCCUPANCY_GRID_FILE_IO_H_
#include <string>

#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>

namespace occupancy_grid_file_io {
// Save an occupancy grid as two files:
// one pgm file contains the cells data and one yaml file which contains
// the map metadata.
// Format of both files are compatible with the existing ROS node map_server.
// See http://wiki.ros.org/map_server for more info.
// @param map_name Name of the map. Both the pgm and yaml files get this name.
// @param map_uuid Uuid of the localization map corresponding to the
// occupancy grid. Can be empty if not used.
// @param map_directory Directory where both pgm and yaml files are saved.
// @param occupancy_grid Occupancy grid to save.
bool SaveOccupancyGridToFiles(
    const std::string& map_name, const std::string& map_uuid,
    const std::string& map_directory, const nav_msgs::OccupancyGrid& occupancy_grid);

// Save data from an occupancy grid in a pgm file.
// Format is compatible with the existing ROS node map_server.
// See http://wiki.ros.org/map_server for more info.
// @param map_name Name of the map. The file gets this name.
// @param map_directory Directory where the file is saved.
// @param occupancy_grid Occupancy grid to save.
bool SaveOccupancyGridDataToPgmFile(
    const std::string& map_name, const std::string& map_directory,
    const nav_msgs::OccupancyGrid& occupancy_grid);

// Save metadata from an occupancy grid in a yaml file.
// Format is compatible with the existing ROS node map_server.
// See http://wiki.ros.org/map_server for more info.
// @param map_name Name of the map. The file gets this name.
// @param map_uuid Uuid of the localization map corresponding to the
// occupancy grid. Can be empty if not used.
// @param map_directory Directory where the file is saved.
// @param map_metadata Occupancy grid metadata to save.
bool SaveOccupancyGridMetadataToYamlFile(
    const std::string& map_name, const std::string& map_uuid,
    const std::string& map_directory, const nav_msgs::MapMetaData& map_metadata);

// Load an occupancy grid from two files:
// one pgm file contains the cells data and one yaml file which contains
// the map metadata.
// @param map_name Name of the map, i.e. of both files.
// @param map_directory Directory where both pgm and yaml files are located.
// @param occupancy_grid Loaded occupancy grid.
// @param map_uuid Uuid of the localization map corresponding to the
// occupancy grid.
bool LoadOccupancyGridFromFiles(
    const std::string&  map_name, const std::string& map_directory,
    nav_msgs::OccupancyGrid* occupancy_grid, std::string* map_uuid);

// Load an occupancy grid data from a pgm file.
// @param map_name Name of the map, i.e. of the file.
// @param map_directory Directory where the file is located.
// @param negate true if blacker pixels should be considered free, and whiter
// pixels occupied.
// @param occupied_threshold Threshold between 0 and 1. Greater values are
// considered as occupied.
// @param free_threshold Threshold between 0 and 1. Smaller values are
// considered as free.
// @param occupancy_grid Loaded occupancy grid.
bool LoadOccupancyGridDataFromPgmFile(
    const std::string&  map_name, const std::string& map_directory,
    bool negate, double occupied_threshold, double free_threshold,
    nav_msgs::OccupancyGrid* occupancy_grid);

// Load an occupancy grid metadata from a yaml file.
// @param map_name Name of the map, i.e. of the file.
// @param map_directory Directory where the file is located.
// @param map_metadata Loaded occupancy grid metadata.
// @param negate true if blacker pixels should be considered free, and whiter
// pixels occupied.
// @param occupied_threshold Threshold between 0 and 1. Greater values are
// considered as occupied.
// @param free_threshold Threshold between 0 and 1. Smaller values are
// considered as free.
// @param map_uuid Uuid of the localization map corresponding to the
// occupancy grid.
bool LoadOccupancyGridMetadataFromYamlFile(
    const std::string&  map_name, const std::string& map_directory,
    nav_msgs::MapMetaData* map_metadata, int* negate,
    double* occupied_threshold, double* free_threshold, std::string* map_uuid);
} // namespace occupancy_grid_file_io
#endif  // OCCUPANCY_GRID_FILE_IO_H_
