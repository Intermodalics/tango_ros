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
#include "tango_ros_util.h"

#include <string.h>

#include <android/log.h>

namespace tango_ros_util {
void LOGI(const char *msg, ...) {
  va_list args;
  va_start(args, msg);
  __android_log_vprint(ANDROID_LOG_INFO, kLogTag, msg, args);
  va_end(args);
}

void LOGW(const char *msg, ...) {
  va_list args;
  va_start(args, msg);
  __android_log_vprint(ANDROID_LOG_WARN, kLogTag, msg, args);
  va_end(args);
}

void LOGE(const char *msg, ...) {
  va_list args;
  va_start(args, msg);
  __android_log_vprint(ANDROID_LOG_ERROR, kLogTag, msg, args);
  va_end(args);
}

void LOGD(const char *msg, ...) {
  va_list args;
  va_start(args, msg);
  __android_log_vprint(ANDROID_LOG_DEBUG, kLogTag, msg, args);
  va_end(args);
}

bool InitRos(const char* master_uri, const char* slave_ip) {
  int argc = 3;
  char* master_uri_copy = strdup(master_uri);
  char* slave_ip_copy = strdup(slave_ip);
  char* argv[] = {"nothing_important" , master_uri_copy, slave_ip_copy};
  ros::init(argc, &argv[0], "tango_x_ros");
  LOGI("Master URI: %s", ros::master::getURI().c_str());
  free(master_uri_copy);
  free(slave_ip_copy);
  if (ros::master::check()) {
    LOGI("ROS MASTER IS UP!");
  } else {
    LOGE("NO ROS MASTER.");
    return false;
  }
  return true;
}

bool IsRosOK() {
  return ros::ok();
}
}  // namespace tango_ros_util
