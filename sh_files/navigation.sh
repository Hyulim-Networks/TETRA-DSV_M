#!/bin/bash
map_name=$1
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
source ~/.bashrc
echo $map_name
export ROS_NAMESPACE=TETRA_NS

pkill -9 -f move_base
sleep 1

roslaunch tetraDS_2dnav move_base_tetra.launch map_name:=${map_name}
