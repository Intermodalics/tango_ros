LOCAL_PATH := $(call my-dir)

# These must go in some sort of order like include flags, otherwise they are dropped
# Oh no, need to automate this with catkin somehow....
stlibs := roscpp boost_signals boost_filesystem rosconsole rosconsole_print rosconsole_backend_interface boost_regex xmlrpcpp roscpp_serialization rostime boost_date_time cpp_common boost_system boost_thread console_bridge move_base rotate_recovery global_planner navfn layers boost_iostreams qhullstatic flann_cpp_s flann_cpp_s-gd nodeletlib bondcpp uuid rosbag rosbag_storage roslz4 lz4 topic_tools voxel_grid tf tf2_ros actionlib tf2 orocos-kdl move_slow_and_clear dwa_local_planner clear_costmap_recovery carrot_planner base_local_planner trajectory_planner_ros urdfdom_sensor urdfdom_model_state urdfdom_model urdfdom_world rosconsole_bridge pointcloud_filters laser_scan_filters mean params increment median transfer_function compressed_image_transport cv_bridge image_transport compressed_depth_image_transport amcl_sensors amcl_map amcl_pf stereo_image_proc image_proc image_geometry opencv_imgproc opencv_core opencv_androidcamera opencv_flann opencv_highgui opencv_features2d opencv_calib3d opencv_ml opencv_video opencv_legacy opencv_objdetect opencv_photo opencv_gpu opencv_videostab opencv_ocl opencv_superres opencv_nonfree opencv_stitching opencv_contrib IlmImf libjasper libjpeg libpng libtiff polled_camera camera_info_manager collada_parser geometric_shapes octomap octomath shape_tools random_numbers camera_calibration_parsers costmap_2d laser_geometry message_filters resource_retriever dynamic_reconfigure_config_init_mutex tinyxml class_loader PocoFoundation roslib rospack boost_program_options pcl_ros_filters pcl_ros_io pcl_ros_tf pcl_common pcl_octree pcl_kdtree pcl_search pcl_sample_consensus pcl_filters pcl_io pcl_features pcl_registration pcl_keypoints pcl_ml pcl_segmentation pcl_stereo pcl_tracking pcl_recognition pcl_surface pluginlib pluginlib_tutorials image_transport_plugins nodelet_math yaml-cpp


#shlibs :=

define include_shlib
$(eval include $$(CLEAR_VARS))
$(eval LOCAL_MODULE := $(1))
$(eval LOCAL_SRC_FILES := $$(LOCAL_PATH)/lib/lib$(1).so)
$(eval include $$(PREBUILT_SHARED_LIBRARY))
endef
define include_stlib
$(eval include $$(CLEAR_VARS))
$(eval LOCAL_MODULE := $(1))
$(eval LOCAL_SRC_FILES := $$(LOCAL_PATH)/lib/lib$(1).a)
$(eval include $$(PREBUILT_STATIC_LIBRARY))
endef

#$(foreach shlib,$(shlibs),$(eval $(call include_shlib,$(shlib))))
$(foreach stlib,$(stlibs),$(eval $(call include_stlib,$(stlib))))

include $(CLEAR_VARS)
LOCAL_MODULE    := roscpp_android_ndk
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include
LOCAL_EXPORT_CPPFLAGS := -fexceptions -frtti
#LOCAL_SRC_FILES := dummy.cpp
LOCAL_EXPORT_LDLIBS := $(foreach l,$(shlibs),-l$(l)) -L$(LOCAL_PATH)/lib
#LOCAL_EXPORT_LDLIBS := -lstdc++ #-L$(LOCAL_PATH)/lib
#LOCAL_SHARED_LIBRARIES := $(shlibs)
LOCAL_STATIC_LIBRARIES := $(stlibs) gnustl_static

include $(BUILD_STATIC_LIBRARY)
