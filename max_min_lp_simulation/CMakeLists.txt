cmake_minimum_required(VERSION 2.8.3)
project(max_min_lp_simulation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  max_min_lp_msgs
  max_min_lp_core
  genmsg
)

add_service_files(
  FILES
  MessageRequest.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES max_min_lp_simulation
  CATKIN_DEPENDS roscpp std_msgs max_min_lp_msgs max_min_lp_core message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(motion_primitive_generator src/motion_primitive_generator.cpp)
target_link_libraries(motion_primitive_generator ${catkin_LIBRARIES})
add_executable(max_min_lp_simulation_central_node src/MaxMinLPCentralNode.cpp)
target_link_libraries(max_min_lp_simulation_central_node ${catkin_LIBRARIES})
add_dependencies(max_min_lp_simulation_central_node max_min_lp_simulation_gencpp)
add_executable(max_min_lp_simulation_robot_node src/MaxMinLPRobotNode.cpp)
target_link_libraries(max_min_lp_simulation_robot_node ${catkin_LIBRARIES})
add_dependencies(max_min_lp_simulation_robot_node max_min_lp_simulation_gencpp)
