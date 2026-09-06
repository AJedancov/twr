# LiDAR

<div class="center-table" markdown>

| **Parameter**         | **Value**  |
|-----------------------|------------|
| Number of samples     | 360        |
| Range \[m\]           | 0.05 - 5   |
| Update rate           | 10         |
| Topic name            | `/scan`    |

</div>

**Full Configuration on GitHub:** [`twr_description/urdf/packages/gz.xacro`](https://github.com/AJedancov/twr/blob/jazzy/twr_description/urdf/packages/gz.xacro#L8)

!!! tip "Visualization tip for Gazebo"

    The Gazebo may not display the LiDAR beams by default. To render them, select `Visualize Lidar` plugin in the right top corner (three dots) and refresh list of topics.
