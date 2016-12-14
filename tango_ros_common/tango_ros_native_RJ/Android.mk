LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := tango_ros_node_native

LOCAL_SRC_FILES := tango_ros_node_RJ.cpp
LOCAL_CFLAGS  += -g0 -ggdb --std=c++11 -pthread -fPIC -fexceptions -frtti
LOCAL_LDLIBS := -landroid -llog
LOCAL_STATIC_LIBRARIES += roscpp_android_ndk
include $(BUILD_SHARED_LIBRARY)

$(call import-add-path, $(LOCAL_PATH)/../../third_party)
$(call import-module,roscpp_android_ndk)
