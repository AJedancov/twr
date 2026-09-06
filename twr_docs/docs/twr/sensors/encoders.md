# Encoders

<div class="center-table" markdown>

| **Parameter**     | **Value**       |
|-------------------|-----------------|
| Topic name        | `/joint_states` |

</div>

**Full Configuration on GitHub:** [`twr_description/urdf/packages/ros2_control.xacro`](https://github.com/AJedancov/twr/blob/jazzy/twr_description/urdf/packages/ros2_control.xacro)

Wheel rotation data are obtained from `ros2_control` state interfaces, which read joint dynamics directly from the Gazebo Sim and publish to `/joint_states` via the Joint State Broadcaster.
