# Description

- [rebuild_and_launch.bash](./host/rebuild_and_launch.bash)
  
  Provides the ability to rebuild the project and launch [twr_bringup](../twr_bringup/launch/twr_bringup.launch.py) for the project with one command.
  
  ```shell
  ./scripts/host/run_docker_container.bash
  ```

- [run_docker_container.bash](./host/run_docker_container.bash)

  Provides automatic image creation, xhost configuration and launch container with all necessary parameters for the GUI.
  
  ```shell
  ./scripts/host/run_docker_container.bash
  ```
  > **Note 1**  
  > To add a local user (docker) to the X Server, you must provide the sudo password.


  > **Note 2**  
  > After each Docker image is created, a **dangling image** will be created.
   
  
  They can be removed using the command
  ```shell
  docker image prune
  ```

  OR you can run the script with the `autoclean` parameter:
  ```shell
  ./scripts/run_docker_container.sh autoclean
  ```


- [twr_entrypoint.bash](./docker/twr_entrypoint.bash)

  Entry point script for Docker container. Specified in [Dockerfile](../Dockerfile). Required for automatic configuration of ROS2 packages and should not be executed manually.
