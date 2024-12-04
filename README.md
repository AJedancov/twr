# TWR Project
The project aims to create a basic model of the differential drive robot using ROS2.
This model can serve as a basis for testing control and localization algorithms.

Ubuntu: 24.04  
ROS2: Jazzy  
Gazebo: Harmonic


## Installation
Install dependencies

```bash
 sudo apt install ros-${ROS_DISTRO}-xacro \
                  ros-${ROS_DISTRO}-joint-state-publisher \
                  ros-${ROS_DISTRO}-ros2-control \
                  ros-${ROS_DISTRO}-ros2-controllers \             
```

Install Gazebo for a specific ROS2 version. More details from the [source](https://gazebosim.org/docs/latest/ros_installation/):
```bash
 sudo apt install ros-${ROS_DISTRO}-ros-gz \
                  ros-${ROS_DISTRO}-gz-ros2-control
```

Open the directory where the ros2 workspace will be stored.
Clone repository and build project:
```bash
 git clone https://github.com/AJedancov/twr.git
 cd twr
 colcon build
```

## Model preview in RViz

Use this launch file to see model preview in RViz2:
```bash
 ros2 launch twr_sim rviz.launch.py
```  

![](images/twr_rviz2.png)


<!-- TODO: use custom rviz config file -->


## Start simulation in Gazebo

The `use_sim_time:=True` parameter is set as the default for using Gazebo time.  

Start the simulation in Gazebo:
```bash
 ros2 launch twr_sim sim.launch.py
```

You can choose whether to use RViz2 for the simulation using the `use_rviz2` launch argument (by default `True`).
