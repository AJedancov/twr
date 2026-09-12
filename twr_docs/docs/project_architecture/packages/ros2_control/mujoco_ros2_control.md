# mujoco_ros2_control

:fontawesome-brands-github: [GitHub](https://github.com/ros-controls/mujoco_ros2_control)
:fontawesome-solid-book: [Official Documentation](https://control.ros.org/jazzy/doc/mujoco_ros2_control/doc/index.html)

!!! warning

    Package `mujoco_ros2_control` must be [built from source](https://github.com/ros-controls/mujoco_ros2_control#quick-start), as prebuilt binaries of version 0.0.3 are outdated and incompatible with current versions. (Last updated: Sept 2026)

The `mujoco_ros2_control` package is used to integrate the `ros2_control` framework with MuJoCo simulation.


Unlike the `GazeboSimSystem` from [gz_ros2_control](./gz_ros2_control.md), the `mujoco_ros2_control` package requires explicitly launching Controller Manager as defined in [`twr_control.launch.py`](https://github.com/AJedancov/twr/blob/jazzy/twr_control/launch/twr_control.launch.py#L145).
Controller Manager reads URDF description from `/robot_description` topic and initializes the hardware interfaces defined in `<ros2_control>` tags.

The package provides `MujocoSystemInterface` which is defined in [`twr_description/urdf/packages/ros2_control_mjc.xacro`](https://github.com/AJedancov/twr/tree/jazzy/twr_description/urdf/packages/ros2_control_mjc.xacro) under the `<ros2_control>` tags. 

```xml
...
    <hardware>
      <plugin>mujoco_ros2_control/MujocoSystemInterface</plugin>
      <param name="mujoco_model">
        $(find twr_sim)/mujoco/worlds/empty.xml
      </param>
    </hardware>
...
```

It allows treating MuJoCo as a regular hardware interface within the `ros2_control` framework. As a parameter to the `MujocoSystemInterface`, the `mujoco_model` specifies the path to the MuJoCo XML world model file. Note that the world model also includes the robot model.
