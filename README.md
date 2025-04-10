# TWR Project
The aim of the project is to create a model of a robot with a differential drive based on ROS2.
This robot model can be used for testing and evaluating various control and navigation algorithms in a simulated environment.

**Ubuntu**: 24.04  
**ROS2 Distro**: Jazzy  
**Gazebo**: Harmonic

## Installation
### Install dependencies

```bash
 sudo apt install ros-${ROS_DISTRO}-xacro \
                  ros-${ROS_DISTRO}-joint-state-publisher \            
```

Install [ros2_control](https://github.com/ros-controls/ros2_control) packages:
```bash
 sudo apt install ros-${ROS_DISTRO}-ros2-control \
                  ros-${ROS_DISTRO}-ros2-controllers \             
```

Install [Gazebo](https://github.com/gazebosim) for a specific ROS2 version. More detailed information can be found [here](https://gazebosim.org/docs/latest/ros_installation/).  
In addition, install the ros2_control plugin for Gazebo:
```bash
 sudo apt install ros-${ROS_DISTRO}-ros-gz \
                  ros-${ROS_DISTRO}-gz-ros2-control
```

Install [Nav2](https://github.com/ros-navigation/navigation2) framework packages:
```bash
sudo apt install ros-${ROS_DISTRO}-navigation2 \
                 ros-${ROS_DISTRO}-nav2-bringup
```

Install [robot_localization](https://github.com/cra-ros-pkg/robot_localization) package for nonlinear state estimation:
```bash
sudo apt install ros-${ROS_DISTRO}-robot-localization
```

### Clone and build project

Open the directory where the ros2 workspace will be stored.
Clone repository and build project:
```bash
 git clone https://github.com/AJedancov/twr.git
 cd twr
 colcon build
```


### Source the setup files

Source the setup file from project directory on every new shell you open:
```bash
 # Replace ".bash" with your shell
 # Possible values are: setup.bash, setup.sh, setup.zsh
 source install/setup.bash
```


## Model preview in RViz2

Use this launch file to see model preview in RViz2:
```bash
 ros2 launch twr_bringup rviz2.launch.py
```  

<div align="center">
  <img src="images/twr_rviz2.png" width="450"/>  
</div>

## Start simulation in Gazebo Sim

```bash
 ros2 launch twr_bringup twr_bringup.launch.py
```
Set the `use_sim_time` parameter to use Gazebo time (by default `True`).  
Set the `use_rviz2` to use RViz2 during simulation time (by default `True`).

<p align="center">
  <img src="images/twr_rviz2_nav.png" width="400"/>  
  <img src="images/twr_gazebo_warehouse.png" width="450"/>
</p>


## Control
### Keyboard control

Use the [teleop_twist_keyboard](https://github.com/ros-teleop/teleop_twist_keyboard) package to implement basic keyboard control:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel -p stamped:=True
```


### Set goal point
The robot is capable of moving to a given point in space.
The [Nav2](https://github.com/ros-navigation/navigation2) framework is used to implement the navigation task.  
Use the "2D Goal Pose" function in RViz2 to set the desired position.  

<div align="center">
  <img src="images/twr_rviz2_set_goal.png" width="400"/>  
</div>


## Available sensors
### LIDAR
**Topic name:** `/scan`  
**Number of samples:** 360  
**Range (m):** 0.05 - 5  

(**Tip:** to visualize LIDAR data in Gazebo, you need to activate the plugin `Visualize Lidar` and refresh list of topics.)