LOCAL_PATH := $(call my-dir)
PROJECT_ROOT_FROM_JNI:= ../../../..
PROJECT_ROOT:= $(call my-dir)/../../../..

include $(CLEAR_VARS)
OPENCV_INSTALL_MODULES:=on
OPENCV_CAMERA_MODULES:=off
OPENCV_LIB_TYPE:=STATIC

include /home/intermodalics/TangoApps/TangoxRos/OpenCV_sdk_native/jni/OpenCV.mk
LOCAL_MODULE    := tango_ros_native
LOCAL_SRC_FILES := tango_ros_node.cc tango_ros_util.cc jni_interface.cc
LOCAL_C_INCLUDES += $(LOCAL_PATH)/include
LOCAL_CFLAGS  += -g3 -ggdb --std=c++0x
LOCAL_LDLIBS += -landroid
LOCAL_STATIC_LIBRARIES += roscpp_android_ndk
LOCAL_SHARED_LIBRARIES := tango_client_api tango_support_api
include $(BUILD_SHARED_LIBRARY)

$(call import-add-path, $(PROJECT_ROOT))
$(call import-module,roscpp_android_ndk)
$(call import-module,tango_client_api)
$(call import-module,tango_support_api)