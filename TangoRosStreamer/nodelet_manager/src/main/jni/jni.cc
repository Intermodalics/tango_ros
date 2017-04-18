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
#include <nodelet/loader.h>
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
Java_eu_intermodalics_nodelet_1manager_NodeletManager_execute(
        JNIEnv* env, jobject /*obj*/, jstring master_uri_value,
        jstring host_ip_value, jstring node_name_value,
        /* unused */ jobjectArray remapping_objects_value) {
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

    ros::init(argc, argv, node_name_string.c_str());
    nodelet::Loader loader;
    std::map<std::string, std::string> remappings;
    std::vector<std::string> nodelet_argv;

    LOG(INFO) << "Start loading nodelets.";
    const bool result = loader.load("/tango", "tango_ros_native/TangoRosNode", remappings, nodelet_argv);
    if (!result) {
        LOG(ERROR) << "Problem loading Tango ROS nodelet!";
        return 1;
    }
    LOG(INFO) << "Finished loading nodelets.";

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}

JNIEXPORT jint JNICALL
Java_eu_intermodalics_nodelet_1manager_NodeletManager_shutdown(
        JNIEnv* /*env*/, jobject /*obj*/) {
    return 0;
}

#ifdef __cplusplus
}
#endif
