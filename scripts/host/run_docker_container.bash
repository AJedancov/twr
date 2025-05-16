#!/bin/bash

# Create Docker image according to Dockerfile from project folder
docker build -t twr:jazzy .

# Add local user (in this case `docker`) to access the host's X server
sudo xhost +local:docker

# Create Docker container from twr:jazzy image 
docker run \
  --interactive \
  --tty \
  --rm \
  --net host \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --env DISPLAY \
  --name twr_jazzy_container \
  twr:jazzy

if [[ "$1" = "autoclean" ]]
then
  docker image prune
fi