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
#include <string>

#include <jni.h>

#include <glog/logging.h>
#include <image_transport/publisher_plugin.h>
#include <nodelet/loader.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include "tango_helper.h"

#ifdef __cplusplus
extern "C" {
#endif

JNIEXPORT jboolean JNICALL
Java_eu_intermodalics_nodelet_1manager_TangoInitializationHelper_setBinderTangoService(
    JNIEnv* env, jclass /*class*/, jobject binder) {
  return tango_helper::SetBinder(env, binder);
}

JNIEXPORT jboolean JNICALL
Java_eu_intermodalics_nodelet_1manager_TangoInitializationHelper_isTangoVersionOk(
    JNIEnv* env, jclass /*class*/, jobject activity) {
  return tango_helper::IsTangoVersionOk(env, activity);
}

JNIEXPORT jint JNICALL
Java_eu_intermodalics_nodelet_1manager_TangoNodeletManager_execute(
        JNIEnv* env, jobject /*obj*/, jstring master_uri_value,
        jstring host_ip_value, jstring node_name_value,
        jobjectArray remapping_objects_value) {
    const char* master_uri = env->GetStringUTFChars(master_uri_value, NULL);
    const char* host_ip = env->GetStringUTFChars(host_ip_value, NULL);
    const char* node_name = env->GetStringUTFChars(node_name_value, NULL);
    int argc = 3;
    std::string master_uri_string("__master:=" + std::string(master_uri));
    std::string host_ip_string("__ip:=" + std::string(host_ip));
    const std::string node_name_string(node_name);
    char* argv[] = {"/", &master_uri_string[0], &host_ip_string[0]};

    env->ReleaseStringUTFChars(master_uri_value, master_uri);
    env->ReleaseStringUTFChars(host_ip_value, host_ip);
    env->ReleaseStringUTFChars(node_name_value, node_name);

    std::map<std::string, std::string> remappings;
    if (remapping_objects_value == NULL || env->GetArrayLength(remapping_objects_value) == 0) {
      LOG(INFO) << "No remapping to be done.";
    } else {
      int remappingStringCount = env->GetArrayLength(remapping_objects_value);
      for (int i = 0; i < remappingStringCount; ++i) {
        jstring remap_arg_value = (jstring) (env->GetObjectArrayElement(remapping_objects_value, i));
        const char* remap_arg = env->GetStringUTFChars(remap_arg_value, NULL);
        // Parse remapping argument to extract old and new names.
        // According to ROS doc, the syntax for remapping arguments is: old_name:=new_name.
        // See http://wiki.ros.org/Remapping%20Arguments.
        std::string remap_arg_string = std::string(remap_arg);
        std::string delimiter = ":=";
        size_t delimiter_position = remap_arg_string.find(delimiter);
        if (delimiter_position == std::string::npos) {
          LOG(ERROR) << "Invalid remapping argument: " << remap_arg << ". The correct syntax is old_name:=new_name.";
          return 1;
        }
        std::string remap_old_name = remap_arg_string.substr(0, delimiter_position);
        remap_arg_string.erase(0, delimiter_position + delimiter.length());
        std::string remap_new_name = remap_arg_string;
        remappings.insert(std::pair<std::string, std::string>(remap_old_name, remap_new_name));
        LOG(INFO) << "Remapping " << remap_old_name << " to " << remap_new_name;
        env->ReleaseStringUTFChars(remap_arg_value, remap_arg);
      }
    }

    ros::init(argc, argv, node_name_string.c_str());
    nodelet::Loader loader;
    std::vector<std::string> nodelet_argv;

    LOG(INFO) << "Start loading nodelets.";
    const bool result = loader.load("/tango", "tango_ros_native/TangoRosNodelet", remappings, nodelet_argv);
    if (!result) {
        LOG(ERROR) << "Problem loading Tango ROS nodelet!";
        return 1;
    }
    LOG(INFO) << "Finished loading nodelets.";

    // Check that all necessary plugins are available.
    pluginlib::ClassLoader<image_transport::PublisherPlugin> image_transport_pub_loader("image_transport", "image_transport::PublisherPlugin");
    if (!image_transport_pub_loader.isClassAvailable("image_transport/raw_pub")) {
      LOG(ERROR) << "Plugin image_transport/raw_pub is not available.";
      return 1;
    }
    if (!image_transport_pub_loader.isClassAvailable("image_transport/compressed_pub")) {
      LOG(ERROR) << "Plugin image_transport/compressed_pub is not available.";
      return 1;
    }

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}

JNIEXPORT jint JNICALL
Java_eu_intermodalics_nodelet_1manager_TangoNodeletManager_shutdown(
        JNIEnv* /*env*/, jobject /*obj*/) {
    return 0;
}

#ifdef __cplusplus
}
#endif
