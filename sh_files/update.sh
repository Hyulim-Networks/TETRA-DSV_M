#!/bin/bash

cd ~/catkin_ws/src
git add .
git reset --hard
git pull origin main

rm ~/update.sh
ln ~/catkin_ws/src/sh_files/update.sh ~/update.sh
sudo chmod +x ~/update.sh

rm ~/startup.sh
ln ~/catkin_ws/src/sh_files/startup.sh ~/startup.sh
sudo chmod +x ~/startup.sh

rm ~/mapping.sh
ln ~/catkin_ws/src/sh_files/mapping.sh ~/mapping.sh
sudo chmod +x ~/mapping.sh

rm ~/mapsave.sh
ln ~/catkin_ws/src/sh_files/mapsave.sh ~/mapsave.sh
sudo chmod +x ~/mapsave.sh
