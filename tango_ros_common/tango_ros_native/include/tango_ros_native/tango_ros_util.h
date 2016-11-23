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
#ifndef TANGO_ROS_UTIL_H_
#define TANGO_ROS_UTIL_H_
#include <string.h>

#include <ros/ros.h>

namespace tango_ros_util {
// Initializes ros with the correct arguments.
// @param master_uri, URI of the ros master.
// @param slave_ip, IP address of the device.
// @return returns true if the ros master was found.
bool InitRos(const char* master_uri, const char* slave_ip);
// Returns true if ROS is OK, false if it has shut down.
bool IsRosOK();
} // namespace tango_ros_util
#endif  // TANGO_ROS_UTIL_H_
