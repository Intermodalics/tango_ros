cmake_minimum_required(VERSION 2.8.3)
project(tango_ros_messages)

find_package(catkin REQUIRED COMPONENTS
   message_generation
   std_msgs
   )

add_service_files(
  DIRECTORY srv
  FILES
  GetMapName.srv
  GetMapUuids.srv
  SaveMap.srv
  TangoConnect.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
)

