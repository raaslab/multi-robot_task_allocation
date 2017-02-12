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

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(max_min_lp_msgs
#   src/${PROJECT_NAME}/max_min_lp_msgs.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(max_min_lp_msgs ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(max_min_lp_msgs_node src/max_min_lp_msgs_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(max_min_lp_msgs_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(max_min_lp_msgs_node
#   ${catkin_LIBRARIES}
# )