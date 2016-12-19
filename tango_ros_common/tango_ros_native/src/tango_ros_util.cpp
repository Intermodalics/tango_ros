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

void Execute(const char * master_uri, const char * slave_ip, const char * node_name) {

  if (InitRos(master_uri, slave_ip, node_name) == false) {
    return;
  }

  // Example configuration for debugging
  tango_ros_native::TangoRosNode tangoRosNode(true, false, tango_ros_native::CAMERA_NONE);

  tangoRosNode.OnTangoServiceConnected();

  tangoRosNode.StartPublishing();

}

bool InitRos(const char * master_uri, const char * slave_ip, const char * node_name) {
  int argc = 3;
  char* master_uri_copy = strdup(master_uri);
  char* slave_ip_copy = strdup(slave_ip);
  char* argv[] = {"nothing_important" , master_uri_copy, slave_ip_copy};

  LOG(INFO) << "About to init ros: " << '\n' << "Master: " << master_uri_copy << '\n' << "Slave: " << slave_ip_copy << '\n' << "Name: " << node_name << '\n';

  try {
    ros::init(argc, &argv[0], node_name);   // ros::init is crashing the app - to be debugged
  } catch (std::exception& e) {
    LOG(INFO) << e.what() << '\n';
  }
  LOG(INFO) << "Master URI: " << ros::master::getURI().c_str();
  free(master_uri_copy);
  free(slave_ip_copy);
  if (ros::master::check()) {
    LOG(INFO) << "ROS MASTER IS UP! ";
  } else {
    LOG(ERROR) << "NO ROS MASTER! ";
    return false;
  }
  return true;
}

}  // namespace tango_ros_util
