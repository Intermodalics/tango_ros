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
#include "tango_ros_native/tango_ros_util.h"

#include <string.h>

#include <glog/logging.h>

#include "tango_ros_native/tango_ros_node.h"

namespace tango_ros_util {
bool InitRos(const char* master_uri, const char* host_ip) {
  int argc = 3;
  char* master_uri_copy = strdup(master_uri);
  char* host_ip_copy = strdup(host_ip);
  char* argv[] = {"nothing_important" , master_uri_copy, host_ip_copy};

  LOG(INFO) << "\nMaster: " << master_uri_copy << "\n"
            << "Host: " << host_ip_copy;
  try {
    ros::init(argc, &argv[0], tango_ros_native::NODE_NAME);
  } catch (std::exception& e) {
    LOG(INFO) << e.what() << "\n";
  }
  free(master_uri_copy);
  free(host_ip_copy);
  if (ros::master::check()) {
    LOG(INFO) << "ROS MASTER IS UP! ";
  } else {
    LOG(ERROR) << "NO ROS MASTER! ";
    return false;
  }
  return true;
}

void Execute() {
  ros::Rate loop_rate(30);
  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}

}  // namespace tango_ros_util
