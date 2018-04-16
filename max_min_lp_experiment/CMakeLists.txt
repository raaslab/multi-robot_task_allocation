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

add_executable(max_min_lp_experiment_node src/GreedyCentralNode.cpp)
target_link_libraries(max_min_lp_experiment_node
  ${catkin_LIBRARIES}
)
