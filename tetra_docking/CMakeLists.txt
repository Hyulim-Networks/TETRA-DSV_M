cmake_minimum_required(VERSION 2.8.3)
project(tetra_docking)

add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  message_runtime
  tf
  nav_msgs
  geometry_msgs
  tf2_geometry_msgs
  tf2
  tf2_ros
  move_base
  actionlib_msgs
  actionlib
)


add_action_files(DIRECTORY action FILES DockRobot.action UndockRobot.action)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs  # Or other packages containing msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  LIBRARIES tetra_docking
  CATKIN_DEPENDS roscpp rospy message_generation message_runtime tf2 tf2_geometry_msgs actionlib_msgs
)

###########
## Build ##
###########
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

add_executable(tetra_docking src/tetra_docking.cpp)

add_dependencies(tetra_docking ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tetra_docking ${catkin_LIBRARIES})
