#include <jni.h>
#include "tango_ros_node.h"
#include "tango_ros_util.h"

static std::shared_ptr<tango_ros_node::TangoRosNode> tango_ros;

#ifdef __cplusplus
extern "C" {
#endif
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
Java_eu_intermodalics_tangoxros_JNIInterface_onCreate(JNIEnv* env, jobject, jobject activity) {
  tango_ros.reset(new tango_ros_node::TangoRosNode());
  tango_ros->OnCreate(env, activity);
}

JNIEXPORT void JNICALL
Java_eu_intermodalics_tangoxros_JNIInterface_onTangoServiceConnected(
    JNIEnv* env, jobject, jobject iBinder) {
  tango_ros->OnTangoServiceConnected(env, iBinder);
}

JNIEXPORT void JNICALL
Java_eu_intermodalics_tangoxros_JNIInterface_tangoDisconnect(JNIEnv*, jobject) {
  tango_ros->TangoDisconnect();
}

JNIEXPORT void JNICALL
Java_eu_intermodalics_tangoxros_JNIInterface_publish(JNIEnv* /*env*/, jobject /*obj*/) {
  tango_ros->Publish();
}

#ifdef __cplusplus
}
#endif
