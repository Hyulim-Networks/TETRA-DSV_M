#!/bin/bash
map_name=$1
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
source ~/.bashrc
export ROS_NAMESPACE=TETRA_NS

pkill -9 -f move_base

while pgrep -x move_base > /dev/null; do
 echo "wait for move_base die";
 sleep 1
done

if [ $# -eq 0 ]; then
  roslaunch tetraDS_2dnav move_base_tetra.launch
else
  roslaunch tetraDS_2dnav move_base_tetra.launch map_name:=${map_name}
fi
