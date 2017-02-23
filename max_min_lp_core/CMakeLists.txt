cmake_minimum_required(VERSION 2.8.3)
project(max_min_lp_core)

find_package(catkin REQUIRED COMPONENTS
  max_min_lp_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES max_min_lp_core
 CATKIN_DEPENDS max_min_lp_msgs
 DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/MaxMinLPCore.cpp
  src/MaxMinLPDecentralizedCore.cpp
  src/MaxMinLPSequentialCore.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME} max_min_lp_msgs_generate_messages_cpp)
