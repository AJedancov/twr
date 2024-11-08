### Start simulation

The “use_sim_time:=True” parameter is set as the default for using Gazebo time.
(When using a real robot, you must add “use_sim_time:=False”)

Install Gazebo for ROS2. More details from the [source](https://gazebosim.org/docs/latest/ros_installation/):
```bash
 sudo apt install ros-${ROS_DISTRO}-ros-gz
```


Run the simulation in GAZEBO:
```bash
 ros2 launch twr_sim sim.launch.py
```

To launch RViz2:
```bash
 ros2 launch twr_sim rviz.launch.py
```





