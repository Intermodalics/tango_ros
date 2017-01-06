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

#include <tango_ros_native/tango_ros_util.h>

#include <jni.h>

static tango_ros_util::TangoRosNodeExecutor tango_ros_node_executor;

#ifdef __cplusplus
extern "C" {
#endif

JNIEXPORT jboolean JNICALL
Java_eu_intermodalics_tangoxros_TangoRosNode_setBinderTangoService(
    JNIEnv* env, jobject /*obj*/, jobject binder) {
  return tango_helper::SetBinder(env, binder);
}

JNIEXPORT jboolean JNICALL
Java_eu_intermodalics_tangoxros_TangoRosNode_isTangoVersionOk(
    JNIEnv* env, jobject /*obj*/, jobject activity) {
  return tango_helper::IsTangoVersionOk(env, activity);
}

JNIEXPORT jint JNICALL
Java_eu_intermodalics_tangoxros_TangoRosNode_execute(JNIEnv* env, jobject /*obj*/,
    jstring master_uri_value, jstring host_ip_value, jstring node_name_value,
    jobjectArray remapping_objects_value) {
  const char* master_uri = env->GetStringUTFChars(master_uri_value, NULL);
  const char* host_ip = env->GetStringUTFChars(host_ip_value, NULL);
  const char* node_name = env->GetStringUTFChars(node_name_value, NULL);

  std::string master = "__master:=" + std::string(master_uri);
  std::string host = "__ip:=" + std::string(host_ip);
  std::string node = std::string(node_name);

  env->ReleaseStringUTFChars(master_uri_value, master_uri);
  env->ReleaseStringUTFChars(host_ip_value, host_ip);
  env->ReleaseStringUTFChars(node_name_value, node_name);

  tango_ros_node_executor = tango_ros_util::TangoRosNodeExecutor();
  return tango_ros_node_executor.Execute(master.c_str(), host.c_str(), node.c_str());
}

JNIEXPORT void JNICALL
Java_eu_intermodalics_tangoxros_TangoRosNode_shutdown(JNIEnv* /*env*/, jobject /*obj*/) {
  tango_ros_node_executor.Shutdown();
}

#ifdef __cplusplus
}
#endif
