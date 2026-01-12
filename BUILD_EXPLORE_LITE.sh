#!/bin/bash
# Build script for explore_lite
# This script builds explore_lite while ignoring workspace Nav2 packages

cd ~/turtlebot3_ws
source /opt/ros/humble/setup.bash

echo "Building explore_lite..."
colcon build --packages-select explore_lite \
  --packages-ignore nav2_msgs nav2_voxel_grid nav2_util nav2_lifecycle_manager \
                     nav2_map_server nav2_costmap_2d nav2_ros_common navigation2

echo "Sourcing install directory..."
source install/setup.bash

echo "Build complete! You can now run:"
echo "  ros2 run explore_lite explore"
