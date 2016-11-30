LOCAL_PATH := $(call my-dir)
PROJECT_ROOT:= $(call my-dir)/..

include $(CLEAR_VARS)

LOCAL_MODULE    := tango_ros_android
LOCAL_SRC_FILES := $(LOCAL_PATH)/src/tango_android_helper.cpp
LOCAL_C_INCLUDES += $(LOCAL_PATH)/include
LOCAL_CFLAGS  += -g3 -ggdb --std=c++11 -pthread -fPIC -fexceptions -frtti
LOCAL_LDLIBS += -landroid -lm -llog
LOCAL_STATIC_LIBRARIES += miniglog
LOCAL_SHARED_LIBRARIES := tango_client_api tango_support_api
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include
include $(BUILD_SHARED_LIBRARY)

$(call import-add-path, $(PROJECT_ROOT)/third_party/tango_api)
$(call import-module,tango_client_api)
$(call import-module,tango_support_api)
