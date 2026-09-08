#!/bin/bash

source install/setup.bash
colcon build

ros2 launch twr_bringup twr_bringup.launch.py $@
