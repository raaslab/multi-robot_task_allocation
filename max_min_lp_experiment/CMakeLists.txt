cmake_minimum_required(VERSION 2.8.3)
project(max_min_lp_experiment)

SET(CMAKE_CXX_FLAGS "-std=c++0x")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  genmsg
)

add_service_files(
  FILES
  RobotRequest.srv
  TargetRequest.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES max_min_lp_experiment
  CATKIN_DEPENDS roscpp rospy std_msgs tf message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(central_node src/GreedyCentralNode.cpp)
target_link_libraries(central_node
  ${catkin_LIBRARIES}
)

add_executable(robot_node src/GreedyRobotNode.cpp)
target_link_libraries(robot_node
  ${catkin_LIBRARIES}
)

add_executable(target_node src/TargetNode.cpp)
target_link_libraries(target_node
  ${catkin_LIBRARIES}
)
