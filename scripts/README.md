# Description

- [rebuild_and_launch.bash](./host/rebuild_and_launch.bash)
  
  Provides the ability to rebuild the project and launch [twr_bringup](../twr_bringup/launch/twr_bringup.launch.py) for the project with one command.
  
  ```shell
  ./scripts/host/rebuild_and_launch.bash
  ```

- [run_docker_container.bash](./host/run_docker_container.bash)

  Provides automatic image creation and launch container with all necessary parameters for the GUI.
  ```shell
  ./scripts/host/run_docker_container.bash
  ```

  To visualize GUI from Docker container, you need add a local user (docker) to the host X Server:  
  ```shell
  sudo xhost +local:docker
  ``` 
  
  > **Note**  
  > After each Docker image is created, a **dangling image** will be created.
  
  They can be removed using the command:
  ```shell
  docker image prune
  ```

  OR you can run the script with the `autoclean` parameter:
  ```shell
  ./scripts/run_docker_container.sh autoclean
  ```


- [twr_entrypoint.bash](./docker/twr_entrypoint.bash)

  Entry point script for Docker container. Specified in [Dockerfile](../Dockerfile).   
  Required for automatic configuration of ROS2 packages and should not be executed manually.
