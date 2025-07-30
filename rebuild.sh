#!/bin/bash

echo "Rebuilding package..."

cd ~/ros2_ws
colcon build
source install/setup.bash

echo "Done"
