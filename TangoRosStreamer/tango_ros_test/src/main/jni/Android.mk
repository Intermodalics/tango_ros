LOCAL_PATH := $(call my-dir)
PROJECT_ROOT_FROM_JNI:= ../../../..
PROJECT_ROOT:= $(call my-dir)/../../../..

include $(CLEAR_VARS)
LOCAL_MODULE := tango_ros_test
LOCAL_SRC_FILES := test/tango_ros_api_test.cc
LOCAL_C_INCLUDES += $(LOCAL_PATH)/include
LOCAL_CFLAGS  += -O2 --std=c++11 -pthread -fPIC -fexceptions -frtti -Wunused-parameter -Wunused-variable
LOCAL_LDLIBS += -landroid -lm -llog -lz
LOCAL_STATIC_LIBRARIES += roscpp_android_ndk miniglog googletest_main
LOCAL_WHOLE_STATIC_LIBRARIES := libtango_ros_native
LOCAL_DISABLE_FATAL_LINKER_WARNINGS=true
include $(BUILD_EXECUTABLE)

$(call import-add-path, $(PROJECT_ROOT)/../../tango_ros/third_party)
$(call import-module, roscpp_android_ndk)
$(call import-module, miniglog)

$(call import-add-path, $(PROJECT_ROOT)/../tango_ros_common)
$(call import-module, tango_ros_native)
$(call import-module,third_party/googletest)


