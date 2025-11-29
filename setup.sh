#!/bin/bash

echo "Setup started"
cd /home/clover/NTO_2026/models
mv pipe_small pipe_main /home/clover/catkin_ws/src/sitl_gazebo/models
if [ $? -eq 0 ]; then
    echo "Models have been successfully moved to home/clover/catkin_ws/src/sitl_gazebo/models"
else
    echo "ERROR: $?"
fi


cd /home/clover/NTO_2026/launch
mv aruco.launch clover.launch /home/clover/catkin_ws/src/clover/clover/launch
if [ $? -eq 0 ]; then
    echo "Launch files have been successfully configured"
else
    echo "ERROR: $?"
fi


cd /home/clover/NTO_2026/front-end
mv clover.html /home/clover/catkin_ws/src/clover/clover/www
if [ $? -eq 0 ]; then
    echo "Front-end files have been successfully moved to /home/clover/catkin_ws/src/clover/clover/www"
else
    echo "ERROR: $?"
fi

cd /home/clover/NTO_2026
mv aruco_map.png /home/clover
if [ $? -eq 0 ]; then
    echo "aruco_map.png have been successfully moved to /home/clover"
else
    echo "ERROR: $?"
fi

rm -rf models
rm -rf launch
rm -rf front-end

echo "Downloading "image_geometry" package ..."
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt update
sudo apt install ros-noetic-image-geometry
if [ $? -eq 0 ]; then
    echo "ros-noetic-image-geometry has been downloaded"
else
    echo "ERROR: $?"
fi


echo "Setup was completed"
echo "____________________USE____________________"
echo "python3 genmap.py - generate map"
echo "python3 genmap.py clean - clean world"
echo "python3 flight.py - start mission"
echo "http://localhost/clover/clover.html - WEB"
echo "any questions? - telegram @EPancho8 (main dev)" 
echo "___________________________________________"
