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
#include <jni.h>
#include "tango_ros_node.h"
#include "tango_ros_util.h"

static std::shared_ptr<tango_ros_node::TangoRosNode> tango_ros;

#ifdef __cplusplus
extern "C" {
#endif

static void set_native_publisher_configuration_from_java_publisher_configuration(
    JNIEnv* env, jobject jpublisherConfiguration,
    tango_ros_node::PublisherConfiguration* publisher_configuration) {
  jclass cls = env->GetObjectClass(jpublisherConfiguration);

  jfieldID fidPublishDevicePose = env->GetFieldID(cls, "publishDevicePose", "Z");
  jboolean publishDevicePose = env->GetBooleanField(jpublisherConfiguration, fidPublishDevicePose);
  publisher_configuration->publish_device_pose = publishDevicePose;

  jfieldID fidPublishPointCloud = env->GetFieldID(cls, "publishPointCloud", "Z");
  jboolean publishPointCloud = env->GetBooleanField(jpublisherConfiguration, fidPublishPointCloud);
  publisher_configuration->publish_point_cloud = publishPointCloud;

  jfieldID fidPublishCamera = env->GetFieldID(cls, "publishCamera",
    "Leu/intermodalics/tangoxros/PublisherConfiguration$CameraType;");
  jobject publishCamera = env->GetObjectField(jpublisherConfiguration, fidPublishCamera);
  jmethodID publishCameraGetValueMethod = env->GetMethodID(
    env->FindClass("eu/intermodalics/tangoxros/PublisherConfiguration$CameraType"),
    "ordinal", "()I");
  jint publishCameraValue = env->CallIntMethod(publishCamera, publishCameraGetValueMethod);
  switch (publishCameraValue) {
    case 0:
      publisher_configuration->publish_camera = tango_ros_node::CameraType::NONE;
      break;
    case 1:
      publisher_configuration->publish_camera = tango_ros_node::CameraType::FISHEYE;
      break;
    case 2:
      publisher_configuration->publish_camera = tango_ros_node::CameraType::COLOR;
      break;
    default:
      publisher_configuration->publish_camera = tango_ros_node::CameraType::NONE;
      break;
  }
}

JNIEXPORT jboolean JNICALL
Java_eu_intermodalics_tangoxros_JNIInterface_initRos(JNIEnv* env, jobject /*obj*/,
    jstring master_uri_value, jstring ip_address_value) {
  const char* master_uri = env->GetStringUTFChars(master_uri_value, NULL);
  const char* ip_address = env->GetStringUTFChars(ip_address_value, NULL);
  return tango_ros_util::InitRos(master_uri, ip_address);
}

JNIEXPORT jboolean JNICALL
Java_eu_intermodalics_tangoxros_JNIInterface_isRosOk(JNIEnv* env, jobject /*obj*/) {
  return tango_ros_util::IsRosOK();
}

JNIEXPORT void JNICALL
Java_eu_intermodalics_tangoxros_JNIInterface_initNode(JNIEnv* env, jobject /*obj*/, jobject activity,
    jobject jpublisherConfiguration) {
  tango_ros_node::PublisherConfiguration publisher_configuration;
  set_native_publisher_configuration_from_java_publisher_configuration(env, jpublisherConfiguration,
    &publisher_configuration);
  tango_ros.reset(new tango_ros_node::TangoRosNode(publisher_configuration));
  tango_ros->CheckTangoVersion(env, activity);
}

JNIEXPORT void JNICALL
Java_eu_intermodalics_tangoxros_JNIInterface_onTangoServiceConnected(
    JNIEnv* env, jobject /*obj*/, jobject iBinder) {
  tango_ros->OnTangoServiceConnected(env, iBinder);
}

JNIEXPORT void JNICALL
Java_eu_intermodalics_tangoxros_JNIInterface_tangoDisconnect(JNIEnv* /*env*/, jobject /*obj*/) {
  tango_ros->TangoDisconnect();
}

JNIEXPORT void JNICALL
Java_eu_intermodalics_tangoxros_JNIInterface_publish(JNIEnv* /*env*/, jobject /*obj*/) {
  tango_ros->Publish();
}

#ifdef __cplusplus
}
#endif
