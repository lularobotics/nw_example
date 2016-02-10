cmake_minimum_required(VERSION 2.8.3)
project(nw_mico_client)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  riemo_move_action
  roscpp
  rospy
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS src
  LIBRARIES
    nw_mico_client
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    message_generation
    riemo_move_action
    roscpp
    rospy
    std_msgs
)

###########
## Build ##
###########
include_directories(
  src
  ${catkin_INCLUDE_DIRS}
)

## NW Mico Client Lib
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/simple_move_robot_client.h
  src/${PROJECT_NAME}/simple_move_robot_client.cpp
)
add_dependencies(
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(nw_mico_client
  ${catkin_LIBRARIES}
)

# Mico simple client example
add_executable(nw_mico_simple_move_client
    src/${PROJECT_NAME}/nw_mico_simple_move_client_main.cpp
)
add_dependencies(
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(nw_mico_simple_move_client
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

