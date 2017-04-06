cmake_minimum_required(VERSION 2.8.3)
project(tango_3d_reconstruction)

find_package(catkin REQUIRED tango_client_api)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS tango_client_api
  LIBRARIES tango_3d_reconstruction
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(tango_3d_reconstruction
  src/tango_3d_reconstruction_api.cpp
  )

target_link_libraries(tango_3d_reconstruction
  ${catkin_LIBRARIES}
  )

install(TARGETS tango_3d_reconstruction
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)