cmake_minimum_required(VERSION 3.0.2)
project(haya_imu_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  tf
  message_generation
)

add_message_files(
  FILES
  ImuData.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_runtime
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)

add_library(
  lite_serial  src/lite_serial.cpp
)

add_executable(haya_imu_node src/haya_imu_node.cpp)
target_link_libraries(haya_imu_node lite_serial ${catkin_LIBRARIES})
