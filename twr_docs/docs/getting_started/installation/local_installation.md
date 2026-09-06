# Local Installation

## Prerequisites

<div class="center-table" markdown>

| **Software**       | **Version**  |
|--------------------|--------------|
| Linux Distro       | Ubuntu 24.04 |
| ROS 2 Distro       | Jazzy        |
| Gazebo Sim         | Harmonic     |

</div>

!!! Note 

    Refer to the official [Gazebo documentation](https://gazebosim.org/docs/latest/ros_installation/) to install a version compatible with the specific ROS 2  distribution.

## 1. Clone repository

Clone the project from GitHub:

```shell
git clone https://github.com/AJedancov/twr.git
cd twr
```

## 2. Install dependencies

Install all required dependencies using rosdep as shown below:

```shell
sudo apt-get update
rosdep init
rosdep update \
    --rosdistro ${ROS_DISTRO}
rosdep install -y \
    --from-paths . \
    --rosdistro ${ROS_DISTRO} \
    --ignore-src
```

## 3. Build project

Build the project using the following command:

```shell
colcon build
```

Once it is built, continue with [usage examples](../usage.md).
