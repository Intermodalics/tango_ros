cmake_minimum_required(VERSION 2.8.3)
project(tango_ros_native)
add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-Wno-unused-parameter -Wno-unused-variable")

find_package(catkin REQUIRED COMPONENTS
   dynamic_reconfigure
   glog_catkin
   image_geometry
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
  CATKIN_DEPENDS dynamic_reconfigure glog_catkin image_geometry roscpp sensor_msgs tango_3d_reconstruction tango_client_api tango_ros_messages tango_support_api tf tf2_ros
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

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}_test test/test_tango_ros_msg.cpp)
  target_link_libraries(${PROJECT_NAME}_test tango_ros_native)
endif()
