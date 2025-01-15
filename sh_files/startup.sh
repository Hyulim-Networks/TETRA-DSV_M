#!/bin/bash
export DISPLAY=:0

source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
source ~/cartographer_ws/devel_isolated/setup.bash

while ! ip route get 1.1.1.1 | grep -oP 'src \K\S+'; do
 echo "wait for interface up try in 1 second";
 sleep 1
done

export ROS_IP=$(ip route get 1.1.1.1 | grep -oP 'src \K\S+')
export ROS_MASTER_URI=http://$ROS_IP:11311
export ROS_NAMESPACE=TETRA_NS
export ROS_HOSTNAME=$ROS_IP

rosclean purge -y

sleep 1
sudo pm2 stop all
cd /home/tetra/work/tetra-single-api
sudo pm2 start npm -- run start $ROS_NAMESPACE # 멀티관제시 주석처리

roslaunch tetraDS_2dnav tetra_configuration.launch

