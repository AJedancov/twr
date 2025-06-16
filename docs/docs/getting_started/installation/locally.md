
# Locally

## Prerequisites

| **Software**     | **Version**  |
|------------------|--------------|
| **Linux Distro** | Ubuntu 24.04 |
| **ROS 2 Distro** | Jazzy        |
| **Gazebo Sim**   | Harmonic     |


## Clone repository
Select the directory where the project will be locatedand and clone the project from GitHub:
```shell
git clone https://github.com/AJedancov/twr.git && 
cd twr
```


## Install dependencies

You can install the required dependencies in several ways, choose one of them:

=== "apt-get"

    ```shell
    sudo apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-gz-ros2-control \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-joint-state-publisher
    ```

=== "rosdep"

    ```shell
    sudo apt-get update \
    && rosdep update \
    --rosdistro ${ROS_DISTRO} \
    && rosdep install -y \
    --from-paths . \
    --rosdistro ${ROS_DISTRO} \
    --ignore-src
    ```



<!-- Install Gazebo for a specific ROS2 version. More detailed information can be found [here](https://gazebosim.org/docs/latest/ros_installation/).   -->

## Build project

And finally, build the project:
```shell
colcon build
```
Once you have the project built, you can continue with the [usage](../usage.md) examples.
