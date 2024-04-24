#!/bin/bash
map_name=$1
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
source ~/.bashrc
export ROS_NAMESPACE=TE2222006
roslaunch tetraDS_2dnav tetra_configuration.launch
