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
  LOGI("Ros is initialized");
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
