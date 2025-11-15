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

cd /home/clover/NTO_2026
rm -rf models
rm -rf launch

echo "Setup was completed"
