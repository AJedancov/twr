# gz_ros2_control-system

:fontawesome-brands-github: [GitHub](https://github.com/ros-controls/gz_ros2_control/tree/jazzy)
:fontawesome-solid-book: [Official Documentation](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html)

The `gz_ros2_control-system` plugin integrates the `ros2_control` framework with the Gazebo Sim.

Internally, it instantiates [Controller Manager](https://control.ros.org/jazzy/doc/ros2_control/controller_manager/doc/userdoc.html#controller-manager), which loads the following controllers:

- [diff_drive_controller](../../ros2_control/ros2_controllers/diff_drive_controller.md)
- joint_state_broadcaster

!!! note

    Since Controller Manager is instantiated within the plugin, the [explicit Node launch](https://github.com/AJedancov/twr/blob/jazzy/twr_control/launch/twr_control.launch.py#L117) can be omitted.

This plugin is included in the robot's URDF/Xacro file under the `<gazebo>` tag in [`twr_description/urdf/packages/gz.xacro`](https://github.com/AJedancov/twr/blob/jazzy/twr_description/urdf/packages/gz.xacro#L46):

```xml
<plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
    <parameters> $(find twr_control)/ros2_controllers/controller_manager/config/controller_manager.yaml </parameters>
</plugin>
```

To initialize Controller Manager and the controllers, this plugin reads the configuration from the specified `<parameters>` tag.
