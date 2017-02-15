cmake_minimum_required(VERSION 2.8.3)
project(max_min_lp_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_message_files(
  DIRECTORY msg
  FILES
  general_node.msg
  general_node_array.msg
  layered_node.msg
  layered_node_array.msg
  target_node.msg
  primitive_node.msg
  server_to_robots.msg
  server_to_robots_array.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES
  CATKIN_DEPENDS roscpp std_msgs message_runtime
#  DEPENDS system_lib
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)