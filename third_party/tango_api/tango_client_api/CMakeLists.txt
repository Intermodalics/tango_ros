cmake_minimum_required(VERSION 2.8.3)
project(tango_client_api)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tango_client_api
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(tango_client_api
  src/tango_client_api.cpp
  )

target_link_libraries(tango_client_api
  ${catkin_LIBRARIES}
  )

install(TARGETS tango_client_api
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)