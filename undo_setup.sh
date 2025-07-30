#!/bin/bash

echo "Cleaning ~/.bashrc..."
sed -i '/source \/opt\/ros\/humble\/setup\.bash/d' ~/.bashrc
sed -i '/source ~\/ros2_ws\/install\/setup\.bash/d' ~/.bashrc
sed -i '/export TURTLEBOT3_MODEL=burger/d' ~/.bashrc
sed -i '/export PATH="\$HOME\/.local\/bin:\$PATH"/d' ~/.bashrc
echo "~/.bashrc cleaned."

echo "Removing ROS2-related directories..."

DIRS_TO_REMOVE=(
  "$HOME/ros2_ws"
  "$HOME/turtlebot3_smart_nav"
  "$HOME/ros2_humble"
)

for dir in "${DIRS_TO_REMOVE[@]}"; do
  if [ -d "$dir" ]; then
    echo "Deleting $dir ..."
    rm -rf "$dir"
  else
    echo "Directory not found: $dir"
  fi
done

source ~/.bashrc

echo "Undo complete"
