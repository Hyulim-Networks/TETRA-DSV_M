cmake_minimum_required(VERSION 2.8.3)
project(Joy_tetra)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  roscpp
  serial
  message_generation
  message_runtime
  tf
  nav_msgs
  geometry_msgs
  std_msgs
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
 # std_msgs  
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  LIBRARIES tracking_tetra
  CATKIN_DEPENDS roscpp message_generation message_runtime
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 ${catkin_INCLUDE_DIRS}
)

#add_compile_options(-std=c++11)
add_executable(Joy_tetra src/Joy_tetra_node.cpp)

add_dependencies(Joy_tetra ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Joy_tetra ${catkin_LIBRARIES})

