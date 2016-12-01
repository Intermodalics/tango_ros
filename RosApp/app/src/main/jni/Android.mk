LOCAL_PATH := $(call my-dir)
PROJECT_ROOT_FROM_JNI:= ../../../..
PROJECT_ROOT:= $(call my-dir)/../../../..

include $(CLEAR_VARS)
LOCAL_MODULE    := tango_ros_android_lib
LOCAL_SRC_FILES := jni_interface.cc tango_helper.cc
LOCAL_C_INCLUDES += $(LOCAL_PATH)/include
LOCAL_CFLAGS  += -g3 -ggdb --std=c++11 -pthread -fPIC -fexceptions -frtti
LOCAL_LDLIBS += -landroid -lm -llog
LOCAL_STATIC_LIBRARIES += roscpp_android_ndk
LOCAL_SHARED_LIBRARIES := tango_client_api tango_support_api tango_ros_native tango_ros_android
include $(BUILD_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := test_tango_ros_native
LOCAL_SRC_FILES := test/test_tango_ros_api.cc
LOCAL_CFLAGS  += -g3 -ggdb --std=c++11 -pthread -fPIC -fexceptions -frtti
LOCAL_STATIC_LIBRARIES += roscpp_android_ndk googletest_main
LOCAL_SHARED_LIBRARIES := tango_client_api tango_ros_native
include $(BUILD_EXECUTABLE)

$(call import-add-path, $(PROJECT_ROOT)/../tango_ros_common)
$(call import-module,tango_ros_native)
$(call import-module,third_party/googletest)


