cmake_minimum_required(VERSION 2.8.3)
project(tango_ros_native)
add_compile_options(-std=c++11)
 
find_package(catkin REQUIRED COMPONENTS
   dynamic_reconfigure
   glog_catkin
   roscpp
   sensor_msgs
   tango_client_api
   tango_support_api
   tf
   tf2_ros
   )

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(JNI REQUIRED)
find_package(OpenCV 2 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/Publisher.cfg
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs glog_catkin tango_client_api tango_support_api tf tf2_ros
  DEPENDS Eigen OpenCV
  )
  
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${JNI_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(tango_ros_native SHARED
  src/tango_ros_node.cpp
  src/tango_ros_util.cpp
  )

target_link_libraries(tango_ros_native
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${OpenCV_LIBRARIES}
  )

add_dependencies(tango_ros_native ${PROJECT_NAME}_gencfg)

install(TARGETS tango_ros_native
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}_test_tango_ros test/test_tango_ros_msg.cpp)
  target_link_libraries(${PROJECT_NAME}_test_tango_ros tango_ros_native)
endif()