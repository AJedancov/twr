# mujoco

:fontawesome-brands-github: [GitHub](https://github.com/google-deepmind/mujoco)
:fontawesome-solid-book: [Official Documentation](https://mujoco.readthedocs.io/en/stable/overview.html)

## Modeling

The MuJoCo simulation environments and robot models are provided in:

- [`twr_sim/mujoco/worlds`](https://github.com/AJedancov/twr/tree/jazzy/twr_sim/mujoco/worlds)
- [`twr_description/mjcf/twr.xml`](https://github.com/AJedancov/twr/tree/jazzy/twr_description/mjcf/twr.xml)

Unlike Gazebo, MuJoCo can't dynamically insert robot at runtime (using `create` Node). This means the robot model needs to be included into the selected world at simulation launch time.

!!! note

    The path to the robot model must be defined based on the file structure in the 'install' directory after the colcon build. See an example in [`empty.xml`](https://github.com/AJedancov/twr/tree/jazzy/twr_sim/mujoco/worlds/empty.xml#L22)

## ROS 2 Integration

To connect MuJoCo and ROS 2, the TWR project relies on the [mujoco_ros2_control](../ros2_control/mujoco_ros2_control.md) package. This package simultaneously launches MuJoCo and integrates the `ros2_control` framework into it, thereby providing access to ROS 2.

!!! warning

    Package `mujoco_ros2_control` must be [built from source](https://github.com/ros-controls/mujoco_ros2_control#quick-start), as prebuilt binaries of version 0.0.3 are outdated and incompatible with current versions. (Last updated: Sept 2026)
