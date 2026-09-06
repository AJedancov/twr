# diff_drive_controller

:fontawesome-brands-github: [GitHub](https://github.com/ros-controls/ros2_controllers/tree/jazzy/diff_drive_controller)
:fontawesome-solid-book: [Official Documentation](https://control.ros.org/jazzy/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html)


The `diff_drive_controller` is a bidirectional kinematic converter that bridges high-level navigation and wheel control.

## Wheel Velocity Computation

1. Receives velocity commands from Nav2 controller via `cmd_vel` topic, with linear velocity $v_x$ and angular velocity $\omega_z$.
2. Converts to individual wheel velocities using inverse kinematics:

    $w_{l} = \frac{v_x - 0.5 \omega_z d}{r}$

    $w_{r} = \frac{v_x + 0.5 \omega_z d}{r}$

    where $d$ - wheel separation, $r$ - wheel radius

3. Sends computed velocities to the hardware interface (simulation).

## Odometry Feedback

1. Reads actual wheel velocities from state interfaces (simulation).
2. Calculates robot velocity using forward kinematics:

    $v_x = \frac{w_{l} + w_{r}}{2}$

    $\omega_z = \frac{w_{r} - w_{l}}{d}$

3. Publishes computed odometry to `/diff_drive_controller/odom` topic.

The odometry output is fused with IMU and LiDAR data by `robot_localization` EKF, providing filtered pose estimates back to Nav2 framework.