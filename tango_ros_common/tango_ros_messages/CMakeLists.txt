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
  MapIO.srv
  SaveMap.srv
  TangoConnect.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime std_msgs
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Copy generated header files by dynamic reconfigure, srv,... into local include folder.
# Otherwise these header files can not be found when building for Android.
set(file_names_to_copy
  GetMapName
  GetMapNameRequest
  GetMapNameResponse
  GetMapUuids
  GetMapUuidsRequest
  GetMapUuidsResponse
  MapIO
  MapIORequest
  MapIOResponse
  SaveMap
  SaveMapRequest
  SaveMapResponse
  TangoConnect
  TangoConnectRequest
  TangoConnectResponse
)
set(generated_header_LOCATION  ${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME}/)
set(generated_header_DESTINATION ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/)
foreach(file_name ${file_names_to_copy})
  set(from ${generated_header_LOCATION}${file_name}.h)
  set(to ${generated_header_DESTINATION}${file_name}.h)
  add_custom_command(
    OUTPUT ${to}
    DEPENDS ${from} ${${PROJECT_NAME}_EXPORTED_TARGETS}
    COMMAND cmake -E copy ${from} ${to})
  add_custom_target(copy_${file_name} ALL DEPENDS ${to})
endforeach(file_name)
