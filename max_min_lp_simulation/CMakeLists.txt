cmake_minimum_required(VERSION 2.8.3)
project(max_min_lp_simulation)

SET(CMAKE_CXX_FLAGS "-std=c++0x")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  max_min_lp_msgs
  max_min_lp_core
  genmsg
  tf
  gazebo_msgs
  gazebo_ros
)

find_package(gazebo REQUIRED)

add_service_files(
  FILES
  MessageRequest.srv
  MotionPrimitiveRequest.srv
  GetOdom.srv
  MoveRobot.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  max_min_lp_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES max_min_lp_simulation
  CATKIN_DEPENDS roscpp std_msgs max_min_lp_msgs max_min_lp_core message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

add_library(animated_box SHARED src/animated_box.cc)
target_link_libraries(animated_box ${GAZEBO_LIBRARIES})

add_executable(motion_primitive_generator src/motion_primitive_generator.cpp)
target_link_libraries(motion_primitive_generator ${catkin_LIBRARIES})

add_executable(max_min_lp_simulation_central_node src/MaxMinLPCentralNode.cpp)
target_link_libraries(max_min_lp_simulation_central_node ${catkin_LIBRARIES})
add_dependencies(max_min_lp_simulation_central_node max_min_lp_simulation_gencpp)
add_executable(max_min_lp_simulation_robot_node src/MaxMinLPRobotNode.cpp)
target_link_libraries(max_min_lp_simulation_robot_node ${catkin_LIBRARIES})
add_dependencies(max_min_lp_simulation_robot_node max_min_lp_simulation_gencpp)

add_executable(max_min_lp_simulation_greedy_central_node src/MaxMinLPGreedyCentralNode.cpp)
target_link_libraries(max_min_lp_simulation_greedy_central_node ${catkin_LIBRARIES})
add_dependencies(max_min_lp_simulation_greedy_central_node max_min_lp_simulation_gencpp)
add_executable(max_min_lp_simulation_greedy_robot_node src/MaxMinLPGreedyRobotNode.cpp)
target_link_libraries(max_min_lp_simulation_greedy_robot_node ${catkin_LIBRARIES})
add_dependencies(max_min_lp_simulation_greedy_robot_node max_min_lp_simulation_gencpp)

add_executable(max_min_lp_simulation_central_node_simulation src/MaxMinLPCentralNodeSimulation.cpp)
target_link_libraries(max_min_lp_simulation_central_node_simulation ${catkin_LIBRARIES})
add_dependencies(max_min_lp_simulation_central_node_simulation max_min_lp_simulation_gencpp)
add_executable(max_min_lp_simulation_robot_node_simulation src/MaxMinLPRobotNodeSimulation.cpp)
target_link_libraries(max_min_lp_simulation_robot_node_simulation ${catkin_LIBRARIES})
add_dependencies(max_min_lp_simulation_robot_node_simulation max_min_lp_simulation_gencpp)

add_executable(apply_motion_primitive_node src/apply_motion_primitive.cpp)
target_link_libraries(apply_motion_primitive_node ${catkin_LIBRARIES})
add_dependencies(apply_motion_primitive_node max_min_lp_simulation_gencpp)

add_executable(get_odom_node src/get_odom.cpp)
target_link_libraries(get_odom_node ${catkin_LIBRARIES})
add_dependencies(get_odom_node max_min_lp_simulation_gencpp)

add_executable(get_target_odom_node src/get_target_odom.cpp)
target_link_libraries(get_target_odom_node ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} pthread)
add_dependencies(get_target_odom_node max_min_lp_simulation_gencpp)