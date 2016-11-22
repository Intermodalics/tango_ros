cmake_minimum_required(VERSION 2.8.3)
project(tango_support_api)

find_package(catkin REQUIRED tango_client_api)
find_package(JNI REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  )

include_directories(
  include
  ${JNI_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(tango_support_api
  src/tango_support_api.cpp
  )

target_link_libraries(tango_support_api
  ${catkin_LIBRARIES}
  tango_client_api
  )

install(TARGETS tango_support_api
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)