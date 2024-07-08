#!/bin/bash

source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
source ~/cartographer_ws/devel_isolated/setup.bash
source ~/.bashrc
echo $ROS_NAMESPACE
export ROS_NAMESPACE=TETRA_NS

pkill -9 -f move_base
sleep 1

roslaunch tetraDS_2dnav cartographer_mapping.launch
