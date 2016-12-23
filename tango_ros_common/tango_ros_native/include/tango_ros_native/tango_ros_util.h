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

#include <tango_ros_native/tango_ros_node.h>

namespace tango_ros_util {
// Initializes ros with the given arguments.
// @param master_uri, URI of the ros master.
// @param host_ip, IP address of the device.
// @param node_name, name of the node.
// @return returns true if the ros master was found.
bool InitRos(const char* master_uri, const char* host_ip,
                                   const char* node_name);

class TangoRosNodeExecutor {
 public:
  TangoRosNodeExecutor();
  ~TangoRosNodeExecutor();
  // Init ros and the tango ros node.
  // Enters a loop, exits when ros shutdown.
  // @param master_uri, URI of the ros master.
  // @param host_ip, IP address of the device.
  // @param node_name, name of the node.
  int Execute(const char* master_uri, const char* host_ip, const char* node_name);
  // Stop the tango ros node and disconnect from the tango service.
  void Shutdown();
  // To be removed.
  void UpdatePublisherConfiguration(bool publish_device_pose,
                                    bool publish_point_cloud,
                                    uint32_t publish_camera);

 private:
  std::shared_ptr<tango_ros_native::TangoRosNode> tango_ros_node_;
};
} // namespace tango_ros_util
#endif  // TANGO_ROS_UTIL_H_
