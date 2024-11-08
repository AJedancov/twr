# TWR Project



# Installation
Install dependencies 

ros-<distro name>-<package name>

```bash
 sudo apt install ros-${ROS_DISTRO}-xacro \
                  ros-${ROS_DISTRO}-joint-state-publisher \
                  ros-${ROS_DISTRO}-joint-state-publisher-gui \
                  ros-${ROS_DISTRO}-ros2-control \
                  ros-${ROS_DISTRO}-ros2-controllers \
                  ros-${ROS_DISTRO}-gz-ros2-control 
```

Clone and build repository
```bash
 git clone https://github.com/AZhed/twr.git
 cd twr
 colcon build
```