LOCAL_PATH := $(call my-dir)

# These must go in some sort of order like include flags, otherwise they are dropped
# Oh no, need to automate this with catkin somehow....
stlibs :=  roscpp boost_signals boost_filesystem rosconsole rosconsole_print rosconsole_backend_interface boost_regex xmlrpcpp dynamic_reconfigure_config_init_mutex roscpp_serialization rostime boost_date_time cpp_common boost_system boost_thread console_bridge rotate_recovery tinyxml class_loader PocoFoundation roslib global_planner navfn costmap_2d layers laser_geometry pcl_ros_filters pcl_ros_io pcl_ros_tf pcl_common pcl_octree pcl_kdtree pcl_search pcl_sample_consensus pcl_filters pcl_io pcl_features pcl_registration pcl_surface pcl_tracking pcl_ml pcl_keypoints pcl_segmentation pcl_stereo pcl_recognition boost_iostreams qhullstatic flann_cpp_s flann_cpp_s-gd nodeletlib bondcpp uuid rosbag rosbag_storage boost_program_options roslz4 lz4 topic_tools voxel_grid tf tf2_ros actionlib message_filters tf2 move_slow_and_clear dwa_local_planner clear_costmap_recovery carrot_planner base_local_planner trajectory_planner_ros robot_state_publisher_solver tf_conversions kdl_conversions kdl_parser orocos-kdl urdfdom_sensor urdfdom_model_state urdfdom_model urdfdom_world rosconsole_bridge moveit_exceptions moveit_background_processing moveit_kinematics_base moveit_robot_model moveit_transforms moveit_robot_state moveit_robot_trajectory moveit_planning_interface moveit_collision_detection moveit_collision_detection_fcl moveit_kinematic_constraints moveit_planning_scene moveit_constraint_samplers moveit_planning_request_adapter moveit_profiler moveit_trajectory_processing moveit_distance_field moveit_kinematics_metrics moveit_dynamics_solver geometric_shapes octomap octomath shape_tools eigen_conversions random_numbers srdfdom pointcloud_filters laser_scan_filters mean params increment median transfer_function interactive_markers compressed_image_transport cv_bridge image_transport compressed_depth_image_transport amcl_sensors amcl_map amcl_pf depth_image_proc stereo_image_proc image_proc image_geometry polled_camera camera_info_manager pluginlib_tutorials nodelet_math collada_parser camera_calibration_parsers resource_retriever rospack


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
#LOCAL_EXPORT_LDLIBS := $(foreach l,$(shlibs),-l$(l)) -L$(LOCAL_PATH)/lib
#LOCAL_EXPORT_LDLIBS := -lstdc++ #-L$(LOCAL_PATH)/lib
#LOCAL_SHARED_LIBRARIES := $(shlibs)
LOCAL_STATIC_LIBRARIES := $(stlibs) gnustl_static

include $(BUILD_STATIC_LIBRARY)
