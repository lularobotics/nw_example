cmake_minimum_required(VERSION 2.8.3)
project(nw_mico_client)

# Use C++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  riemo_move_action
  roscpp
  roslib
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
    roslib
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

add_dependencies(
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Mico simple client example
add_executable(nw_mico_simple_move_client
    src/${PROJECT_NAME}/nw_mico_simple_move_client_main.cpp
)
add_dependencies(
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    riemo_move_action_generate_messages_cpp
)
target_link_libraries(nw_mico_simple_move_client
  ${catkin_LIBRARIES}
  yaml-cpp
)

