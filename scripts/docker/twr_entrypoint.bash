#!/bin/bash

source /opt/ros/$ROS_DISTRO/setup.bash

# Build the project
cd $TWR_WS
colcon build

# Ensure that workspaces are sourced in every new shell
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$(whoami)/.bashrc
echo "source $TWR_WS/install/setup.bash" >> /home/$(whoami)/.bashrc
source /home/$(whoami)/.bashrc

# Execute any command specified after entrypoint script
echo "Provided commands: $@"
exec "$@"