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
#include "tango_helper.h"

#include <tango_ros_native/tango_ros_node.h>
#include <tango_ros_native/tango_ros_util.h>

#include <jni.h>

static std::shared_ptr<tango_ros_native::TangoRosNode> tango_ros; // to be removed

#ifdef __cplusplus
extern "C" {
#endif

static void set_native_publisher_configuration_from_java_publisher_configuration(
    JNIEnv* env, jobject jpublisherConfiguration, bool* publish_device_pose,
    bool* publish_point_cloud, uint32_t* publish_camera) {
  jclass cls = env->GetObjectClass(jpublisherConfiguration);

  jfieldID fidPublishDevicePose = env->GetFieldID(cls, "publishDevicePose", "Z");
  jboolean publishDevicePose = env->GetBooleanField(jpublisherConfiguration, fidPublishDevicePose);
  *publish_device_pose = publishDevicePose;

  jfieldID fidPublishPointCloud = env->GetFieldID(cls, "publishPointCloud", "Z");
  jboolean publishPointCloud = env->GetBooleanField(jpublisherConfiguration, fidPublishPointCloud);
  *publish_point_cloud = publishPointCloud;

  jfieldID fidPublishCamera = env->GetFieldID(cls, "publishCamera", "I");
  int publishCamera = env->GetIntField(jpublisherConfiguration, fidPublishCamera);
  *publish_camera = publishCamera;
}

JNIEXPORT jboolean JNICALL
Java_eu_intermodalics_tangoxros_TangoRosNode_setBinderTangoService(
    JNIEnv* env, jobject /*obj*/, jobject iBinder) {
  return tango_helper::SetBinder(env, iBinder);
}

JNIEXPORT void JNICALL
Java_eu_intermodalics_tangoxros_TangoRosNode_execute(JNIEnv* env, jobject /*obj*/,
    jstring master_uri_value, jstring host_ip_value, jstring node_name_value,
    jobjectArray remapping_objects_value) {
  const char* master_uri = env->GetStringUTFChars(master_uri_value, NULL);
  const char* host_ip = env->GetStringUTFChars(host_ip_value, NULL);

  std::string master("__master:=" + std::string(master_uri));
  std::string host("__ip:=" + std::string(host_ip));

  env->ReleaseStringUTFChars(master_uri_value, master_uri);
  env->ReleaseStringUTFChars(host_ip_value, host_ip);

  if (tango_ros_util::InitRos(master.c_str(), host.c_str())) {
    tango_ros.reset(new tango_ros_native::TangoRosNode());
    if (tango_ros->OnTangoServiceConnected()) {
      tango_ros->StartPublishing();
      tango_ros_util::Execute(); // enter while ros::ok loop
    }
  }
}

JNIEXPORT void JNICALL
Java_eu_intermodalics_tangoxros_TangoRosNode_shutdown(JNIEnv *, jobject) {
  tango_ros->StopPublishing();
  tango_ros->TangoDisconnect();
}

JNIEXPORT void JNICALL
Java_eu_intermodalics_tangoxros_TangoRosNode_updatePublisherConfiguration(JNIEnv* env, jobject /*obj*/,
    jobject jpublisherConfiguration) {
  bool publish_device_pose;
  bool publish_point_cloud;
  uint32_t publish_camera;
  set_native_publisher_configuration_from_java_publisher_configuration(env, jpublisherConfiguration,
    &publish_device_pose, &publish_point_cloud, &publish_camera);
  tango_ros->UpdatePublisherConfiguration(publish_device_pose, publish_point_cloud, publish_camera);
}

#ifdef __cplusplus
}
#endif