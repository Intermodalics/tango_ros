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
#ifndef NAVIGATION_MAP_FILE_IO_H_
#define NAVIGATION_MAP_FILE_IO_H_
#include <string>

#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>

namespace navigation_map_file_io {
//
bool SaveOccupancyGridToNavigationMap(
    const std::string& map_name, const std::string& map_uuid,
    const std::string& map_directory, const nav_msgs::OccupancyGrid& occupancy_grid);
//
bool SaveOccupancyGridDataToPgmFile(
    const std::string& map_name, const std::string& map_directory,
    const nav_msgs::OccupancyGrid& occupancy_grid);
//
bool SaveOccupancyGridMetadataToYamlFile(
    const std::string& map_name, const std::string& map_uuid,
    const std::string& map_directory, const nav_msgs::MapMetaData& map_metadata);

//
bool LoadOccupancyGridFromNavigationMap(
    const std::string&  map_name, const std::string& map_uuid,
    const std::string& map_directory, nav_msgs::OccupancyGrid* occupancy_grid);

//
bool LoadOccupancyGridDataFromPgmFile(
    const std::string&  map_name, const std::string& map_directory,
    bool negate, double occupied_threshold, double free_threshold,
    nav_msgs::OccupancyGrid* occupancy_grid);

//
bool LoadOccupancyGridMetadataFromYamlFile(
    const std::string&  map_name, const std::string& map_uuid,
    const std::string& map_directory, nav_msgs::MapMetaData* map_metadata,
    int* negate, double* occupied_threshold, double* free_threshold);
} // namespace navigation_map_file_io
#endif  // NAVIGATION_MAP_FILE_IO_H_
