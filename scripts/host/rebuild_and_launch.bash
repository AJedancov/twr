#!/bin/bash

source install/setup.bash
colcon build

if [[ "$1" = "test" ]];
then
  exit 0
else
  ros2 launch twr_bringup twr_bringup.launch.py
fi