#!/bin/bash

ROS_DISTRO="jazzy"
TWR_WS="/twr"

source /opt/ros/$ROS_DISTRO/setup.bash

# Build the project
cd $TWR_WS
colcon build

# Ensure that workspaces are sourced in every new shell
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc
echo "source $TWR_WS/install/setup.bash" >> /root/.bashrc
source /root/.bashrc

# Execute any command specified after entrypoint script
echo "Provided commands: $@"
exec "$@"