#include "tango_ros_util.h"

#include <string.h>

#include <glog/logging.h>

namespace tango_ros_util {

bool InitRos(const char* master_uri, const char* slave_ip) {
  int argc = 3;
  char* master_uri_copy = strdup(master_uri);
  char* slave_ip_copy = strdup(slave_ip);
  char* argv[] = {"nothing_important" , master_uri_copy, slave_ip_copy};
  ros::init(argc, &argv[0], "tango_x_ros");
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

bool IsRosOK() {
  return ros::ok();
}
}  // namespace tango_ros_util
