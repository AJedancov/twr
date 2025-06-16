
# Usage

Source the setup file from project directory on every new shell you open:
```sh
source install/setup.bash # (1)!
```

1.  Replace ".bash" with your shell.   
    Possible values are: 
    - setup.bash   
    - setup.sh   
    - setup.zsh

???+ note "Optional"     
    For convenience, you can include this command in the bash file for automatic execution each time you open a new terminal. Run the command from `twr` directory:
    
    ```shell
    echo "source $(pwd)/install/setup.bash" >> ~/.bashrc
    ```


## Model preview in RViz2

Use this launch file to see model preview in RViz2:
```bash
ros2 launch twr_bringup rviz2.launch.py
```  

As a result, the RViz2 will be launched with a robot model without simulation:

<p align="center">
  <img src="/../assets/screenshots/twr_rviz2.png" width="600"/>  
</p>



## Start simulation in Gazebo Sim

The following command will launch the main bringup file for the robot in the simulation:
```bash
ros2 launch twr_bringup twr_bringup.launch.py
```

Alternatively, you can use a helper script to get started quickly:
```shell
chmod 700 scripts/host/rebuild_and_launch.bash && 
./scripts/host/rebuild_and_launch.bash # (1)!
```

1. For more information, see [scripts](../documentation/project_structure/scripts.md) description.  

<!-- TODO: For more information about bringup launch files, see here -->

<!-- Set the `use_sim_time` parameter to use Gazebo time (by default `True`).  
Set the `use_rviz2` to use RViz2 during simulation time (by default `True`). -->

<div align="center">
  <img src="/../assets/screenshots/twr_rviz2_nav.png" width="500"/>  
  <img src="/../assets/screenshots/twr_gazebo_warehouse.png" width="500"/>
</div>


## Control
### Keyboard control

Use the [teleop_twist_keyboard](https://github.com/ros-teleop/teleop_twist_keyboard) package to implement basic keyboard control:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -r /cmd_vel:=/diff_drive_controller/cmd_vel \
  -p stamped:=True
```


### Set goal point

This function is implemented based on the [Nav2](https://github.com/ros-navigation/navigation2) framework.  
Use the `2D Goal Pose` function in RViz2 to set the desired position: 
<div align="center">
  <img src="/assets/screenshots/rviz2_goal_pose.png" width="400"/>  
</div>

Now you can choose any position and orientation of the robot on the presented map:

<div align="center">
  <img src="/assets/screenshots/twr_rviz2_set_goal.png" width="500"/>  
</div>
