
# Scripts description

## [rebuild_and_launch.bash](https://github.com/AJedancov/twr/blob/master/scripts/host/rebuild_and_launch.bash)
  
  Provides the ability to rebuild the project and launch [twr_bringup](https://github.com/AJedancov/twr/blob/master/twr_bringup/launch/twr_bringup.launch.py) with one command.
  
  ```shell
  ./scripts/host/rebuild_and_launch.bash
  ```

## [run_docker_container.bash](https://github.com/AJedancov/twr/blob/master/scripts/host/run_docker_container.bash)

  Provides automatic image creation and launch container with all necessary parameters for the GUI.
  ```shell
  ./scripts/host/run_docker_container.bash
  ```

  To visualize GUI from Docker container, you need add a local user (docker) to the host X Server:  
  ```shell
  sudo xhost +local:docker
  ``` 
  
!!! note
    After each image is created, a **dangling image** will be left.
  
  They can be removed using the command:
  ```shell
  docker image prune
  ```

  Or you can run the script with the `autoclean` parameter:
  ```shell
  ./scripts/run_docker_container.sh autoclean
  ```


## [twr_entrypoint.bash](https://github.com/AJedancov/twr/blob/master/scripts/docker/twr_entrypoint.bash)

  Entry point script for Docker container. Specified in [Dockerfile](https://github.com/AJedancov/twr/blob/master/Dockerfile).   
  Required for automatic configuration of ROS2 packages and should not be executed manually.
