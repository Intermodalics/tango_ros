LOCAL_PATH := $(call my-dir)
PROJECT_ROOT:= $(call my-dir)/..

include $(CLEAR_VARS)

LOCAL_MODULE    := tango_ros_native
LOCAL_SRC_FILES := $(LOCAL_PATH)/src/tango_ros_node.cpp $(LOCAL_PATH)/src/tango_ros_util.cpp
LOCAL_C_INCLUDES += $(LOCAL_PATH)/include ${LOCAL_PATH}/../tango_ros_messages/include
LOCAL_CFLAGS  += -O2 --std=c++11 -pthread -fPIC -fexceptions -frtti -Wunused-parameter -Wunused-variable
LOCAL_LDLIBS += -landroid -lm -llog -lz
LOCAL_WHOLE_STATIC_LIBRARIES := libimage_transport_plugins libcompressed_image_transport
LOCAL_STATIC_LIBRARIES += roscpp_android_ndk miniglog
LOCAL_SHARED_LIBRARIES := tango_3d_reconstruction tango_client_api tango_support_api
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include ${LOCAL_PATH}/../tango_ros_messages/include
LOCAL_DISABLE_FATAL_LINKER_WARNINGS=true
include $(BUILD_SHARED_LIBRARY)

$(call import-add-path, $(PROJECT_ROOT)/../third_party)
$(call import-module,roscpp_android_ndk)
$(call import-module,miniglog)
$(call import-add-path, $(PROJECT_ROOT)/../third_party/tango_api)
$(call import-module,tango_3d_reconstruction)
$(call import-module,tango_client_api)
$(call import-module,tango_support_api)
