cmake_minimum_required(VERSION 2.8.3)
project(tango_ros_native)
add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-Wno-unused-parameter -Wno-unused-variable")

find_package(catkin REQUIRED COMPONENTS
   dynamic_reconfigure
   glog_catkin
   image_geometry
   image_transport
   roscpp
   sensor_msgs
   tango_3d_reconstruction
   tango_client_api
   tango_ros_messages
   tango_support_api
   tf
   tf2_ros
   )
find_package(JNI REQUIRED)


# Use OpenCV3 if we are using ROS Kinetic (which is the default for that distro)
if( $ENV{ROS_DISTRO} STRLESS "kinetic")
  find_package(OpenCV 2 REQUIRED)
else()
  find_package(OpenCV 3 REQUIRED)
endif()

generate_dynamic_reconfigure_options(
  cfg/Publisher.cfg
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS dynamic_reconfigure glog_catkin image_geometry image_transport roscpp sensor_msgs tango_3d_reconstruction tango_client_api tango_ros_messages tango_support_api tf tf2_ros
  LIBRARIES tango_ros_native
  DEPENDS OpenCV
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${JNI_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)

add_library(tango_ros_native SHARED
  src/tango_ros_conversions_helper.cpp
  src/tango_ros_node.cpp
  )

target_link_libraries(tango_ros_native
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  )

add_dependencies(tango_ros_native ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

install(TARGETS tango_ros_native
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Copy generated header files by dynamic reconfigure, srv,... into local include folder.
# Otherwise these header files can not be found when building for Android.
set(file_names_to_copy
  PublisherConfig
)
set(generated_header_LOCATION  ${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME}/)
set(generated_header_DESTINATION ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/)
foreach(file_name ${file_names_to_copy})
  set(from ${generated_header_LOCATION}${file_name}.h)
  set(to ${generated_header_DESTINATION}${file_name}.h)
  add_custom_command(
    OUTPUT ${to}
    DEPENDS ${from}
    COMMAND cmake -E copy ${from} ${to})
  add_custom_target(copy_${file_name} ALL DEPENDS ${to})
endforeach(file_name)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}_test test/test_tango_ros_msg.cpp)
  target_link_libraries(${PROJECT_NAME}_test tango_ros_native)
endif()
