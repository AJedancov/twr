#!/bin/bash

set -e

echo "TWR - Two-Wheeled Robot Setup & Launch"
echo "======================================"
echo ""

echo "Installing dependencies..."
sudo apt-get update

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    rosdep init
else
    echo "rosdep already initialized, skipping rosdep init..."
fi

rosdep update \
    --rosdistro ${ROS_DISTRO}
rosdep install -y \
    --from-paths . \
    --rosdistro ${ROS_DISTRO} \
    --ignore-src

echo "Building project..."
colcon build

echo "Launching TWR..."
source install/setup.sh
ros2 launch twr_bringup twr_bringup.launch.py
