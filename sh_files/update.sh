#!/bin/bash

source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
source ~/cartographer_ws/devel_isolated/setup.bash

pkill -9 -f rviz
pkill -9 -f move_base
pkill -9 -f rosmaster

while pgrep -x move_base > /dev/null; do
 echo "wait for move_base die";
 sleep 1
done
while pgrep -x rosmaster > /dev/null; do
 echo "wait for ros master die";
 sleep 1
done

cd ~/catkin_ws/src
git add .
git reset --hard
git pull origin main

while [ -e ~/catkin_ws/build ]; do
 rm -rf ~/catkin_ws/build
 echo "wait for delete ~/catkin_ws/build";
 sleep 1
done

while [ -e ~/catkin_ws/devel ]; do
 rm -rf ~/catkin_ws/devel
 echo "wait for delete ~/catkin_ws/devel";
 sleep 1
done

while [ -e ~/catkin_ws/.catkin_workspace ]; do
 rm -rf ~/catkin_ws/.catkin_workspace
 echo "wait for delete ~/catkin_ws/.catkin_workspace";
 sleep 1
done

cd ~/catkin_ws
catkin_make

rm -rf ~/update.sh
ln ~/catkin_ws/src/sh_files/update.sh ~/update.sh
chmod +x ~/update.sh

rm -rf ~/startup.sh
ln ~/catkin_ws/src/sh_files/startup.sh ~/startup.sh
chmod +x ~/startup.sh

rm -rf ~/mapping.sh
ln ~/catkin_ws/src/sh_files/mapping.sh ~/mapping.sh
chmod +x ~/mapping.sh

rm -rf ~/mapsave.sh
ln ~/catkin_ws/src/sh_files/mapsave.sh ~/mapsave.sh
chmod +x ~/mapsave.sh
