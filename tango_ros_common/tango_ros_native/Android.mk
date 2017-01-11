LOCAL_PATH := $(call my-dir)
PROJECT_ROOT:= $(call my-dir)/..

include $(CLEAR_VARS)

OPENCV_INSTALL_MODULES:=on
OPENCV_CAMERA_MODULES:=off
OPENCV_LIB_TYPE:=STATIC
include $(PROJECT_ROOT)/../third_party/OpenCV_sdk_native/jni/OpenCV.mk

LOCAL_MODULE    := tango_ros_native
LOCAL_SRC_FILES := $(LOCAL_PATH)/src/tango_ros_node.cpp $(LOCAL_PATH)/src/tango_ros_util.cpp
LOCAL_C_INCLUDES += $(LOCAL_PATH)/include
LOCAL_CFLAGS  += -g3 -ggdb --std=c++11 -pthread -fPIC -fexceptions -frtti -Wunused-parameter -Wunused-variable
LOCAL_LDLIBS += -landroid -lm -llog
LOCAL_STATIC_LIBRARIES += roscpp_android_ndk miniglog
LOCAL_SHARED_LIBRARIES := tango_client_api tango_support_api
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include
LOCAL_DISABLE_FATAL_LINKER_WARNINGS=true
include $(BUILD_SHARED_LIBRARY)

include $(PROJECT_ROOT)/../third_party/miniglog/Android.mk
$(call import-add-path, $(PROJECT_ROOT)/../third_party)
$(call import-module,roscpp_android_ndk)
$(call import-add-path, $(PROJECT_ROOT)/../third_party/tango_api)
$(call import-module,tango_client_api)
$(call import-module,tango_support_api)
