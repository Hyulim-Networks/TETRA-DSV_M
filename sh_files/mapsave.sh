#!/bin/bash
map_name=$1
export ROS_MASTER_URI=http://localhost:11311
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
roscd tetraDS_2dnav/maps/
rosrun map_server map_saver --occ 55 --free 45 -f ${map_name}

