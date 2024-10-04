#!/bin/bash

source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
source ~/cartographer_ws/devel_isolated/setup.bash

cd ~/catkin_ws/src
git add .
git reset --hard
git pull origin main

rm -rf ~/catkin_ws/build
rm -rf ~/catkin_ws/devel
rm -rf ~/catkin_ws/.catkin_workspace
cd ~/catkin_ws
catkin_make

rm -rf ~/update.sh
ln ~/catkin_ws/src/sh_files/update.sh ~/update.sh
sudo chmod +x ~/update.sh

rm -rf ~/startup.sh
ln ~/catkin_ws/src/sh_files/startup.sh ~/startup.sh
sudo chmod +x ~/startup.sh

rm -rf ~/mapping.sh
ln ~/catkin_ws/src/sh_files/mapping.sh ~/mapping.sh
sudo chmod +x ~/mapping.sh

rm -rf ~/mapsave.sh
ln ~/catkin_ws/src/sh_files/mapsave.sh ~/mapsave.sh
sudo chmod +x ~/mapsave.sh
