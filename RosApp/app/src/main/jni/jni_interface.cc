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

static std::shared_ptr<tango_ros_native::TangoRosNode> tango_ros;

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
Java_eu_intermodalics_tangoxros_JNIInterface_initRos(JNIEnv* env, jobject /*obj*/,
    jstring master_uri_value, jstring ip_address_value) {
  const char* master_uri = env->GetStringUTFChars(master_uri_value, NULL);
  const char* ip_address = env->GetStringUTFChars(ip_address_value, NULL);
  return tango_ros_util::InitRos(master_uri, ip_address);
}

JNIEXPORT jboolean JNICALL
Java_eu_intermodalics_tangoxros_JNIInterface_initNode(JNIEnv* env, jobject /*obj*/, jobject activity,
    jobject jpublisherConfiguration) {
  bool publish_device_pose;
  bool publish_point_cloud;
  uint32_t publish_camera;
  set_native_publisher_configuration_from_java_publisher_configuration(env, jpublisherConfiguration,
    &publish_device_pose, &publish_point_cloud, &publish_camera);
  tango_ros.reset(new tango_ros_native::TangoRosNode(publish_device_pose, publish_point_cloud,
    publish_camera));
  return tango_helper::IsTangoVersionOk(env, activity);
}

JNIEXPORT jboolean JNICALL
Java_eu_intermodalics_tangoxros_JNIInterface_onTangoServiceConnected(
    JNIEnv* env, jobject /*obj*/, jobject iBinder) {
  return tango_helper::SetBinder(env, iBinder) && tango_ros->OnTangoServiceConnected();
}

JNIEXPORT void JNICALL
Java_eu_intermodalics_tangoxros_JNIInterface_tangoDisconnect(JNIEnv* /*env*/, jobject /*obj*/) {
  tango_ros->TangoDisconnect();
}

JNIEXPORT void JNICALL
Java_eu_intermodalics_tangoxros_JNIInterface_startPublishing(JNIEnv* /*env*/, jobject /*obj*/) {
  tango_ros->StartPublishing();
}

JNIEXPORT void JNICALL
Java_eu_intermodalics_tangoxros_JNIInterface_stopPublishing(JNIEnv* /*env*/, jobject /*obj*/) {
  tango_ros->StopPublishing();
}

JNIEXPORT void JNICALL
Java_eu_intermodalics_tangoxros_JNIInterface_updatePublisherConfiguration(JNIEnv* env, jobject /*obj*/,
    jobject jpublisherConfiguration) {
  bool publish_device_pose;
  bool publish_point_cloud;
  uint32_t publish_camera;
  set_native_publisher_configuration_from_java_publisher_configuration(env, jpublisherConfiguration,
    &publish_device_pose, &publish_point_cloud, &publish_camera);
  tango_ros->UpdatePublisherConfiguration(publish_device_pose, publish_point_cloud, publish_camera);
}

JNIEXPORT void JNICALL Java_eu_intermodalics_tangoxros_TangoRosNode_execute
  (JNIEnv * env, jobject obj, jstring master_uri_value, jstring slave_ip_value, jstring node_name_value, jobjectArray) {
	// Implementation pending
	// This function shall deal with initializing Ros, the node itself, and the Tango Service.
	// Then, it should start a running loop publishing the data required by the user.
}

JNIEXPORT void JNICALL Java_eu_intermodalics_tangoxros_TangoRosNode_shutdown
  (JNIEnv *, jobject) {
	// Implementation pending
	// shutdown should stop publishing, and disconnect from Tango Service.
}

#ifdef __cplusplus
}
#endif
