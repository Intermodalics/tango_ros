cmake_minimum_required(VERSION 2.8.3)
project(tango_support)

find_package(catkin REQUIRED tango_client_api)
find_package(JNI REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS tango_client_api
  LIBRARIES tango_support
  )

include_directories(
  include
  ${JNI_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(tango_support
  src/tango_support.cpp
  )

target_link_libraries(tango_support
  ${catkin_LIBRARIES}
  )

install(TARGETS tango_support
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)