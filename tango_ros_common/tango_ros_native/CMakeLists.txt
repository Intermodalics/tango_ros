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
   tango_client_api
   tf
   tf2_ros
   )

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
  CATKIN_DEPENDS dynamic_reconfigure glog_catkin image_geometry roscpp sensor_msgs tango_client_api tf tf2_ros
  LIBRARIES tango_ros_native
  DEPENDS OpenCV
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)

add_library(tango_ros_native SHARED
  src/tango_ros_node.cpp
  src/tango_ros_util.cpp
  )

target_link_libraries(tango_ros_native
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  )

add_dependencies(tango_ros_native ${PROJECT_NAME}_gencfg)

install(TARGETS tango_ros_native
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Copy generated header files by dynamic reconfigure into local include folder.
# Otherwise these header files can not be found when building for Android.
set(dynamic_reconfigure_header_LOCATION ${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME}/PublisherConfig.h)
set(dynamic_reconfigure_header_DESTINATION ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/PublisherConfig.h)
add_custom_command(
  OUTPUT ${dynamic_reconfigure_header_DESTINATION}
  DEPENDS ${dynamic_reconfigure_header_LOCATION}
  COMMAND cmake -E copy ${dynamic_reconfigure_header_LOCATION} ${dynamic_reconfigure_header_DESTINATION}
)
add_custom_target(copy ALL DEPENDS ${dynamic_reconfigure_header_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}_test_tango_ros test/test_tango_ros_msg.cpp)
  target_link_libraries(${PROJECT_NAME}_test_tango_ros tango_ros_native)
endif()
