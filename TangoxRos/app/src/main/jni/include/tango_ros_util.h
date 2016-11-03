#ifndef TANGO_ROS_UTIL_H_
#define TANGO_ROS_UTIL_H_
#include <string.h>

#include <ros/ros.h>

namespace tango_ros_util {
static const char* kLogTag = "TANGO_ROS_NATIVE";

void LOGI(const char *msg, ...);
void LOGW(const char *msg, ...);
void LOGE(const char *msg, ...);
void LOGD(const char *msg, ...);

bool InitRos(const char* master_uri, const char* slave_ip);
bool IsRosOK();
} // namespace tango_ros_util
#endif  // TANGO_ROS_UTIL_H_
