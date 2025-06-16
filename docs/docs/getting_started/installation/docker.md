# Docker


## Prerequisites
Follow the [instructions](https://docs.docker.com/engine/install/) to install Docker from the official website.

## Clone repository
Open the directory where the project will be locatedand and clone repository:
```shell
git clone https://github.com/AJedancov/twr.git && 
cd twr
```

## Build Docker image

First you need to create a Docker image.
```shell
docker build -t twr:jazzy .
```
This command will create an image according to the [Dockerfile](https://github.com/AJedancov/twr/blob/master/Dockerfile) located at the root of the project. 


To render the GUI from a Docker container, you need to add a local user (docker) to the host X server:
```shell
sudo xhost +local:docker
```

!!! note
    Currently, the GUI is only displayed when using the X Window System. 


## Run Docker container

And finally, you can run the container:

```shell
docker run -it \
--rm \
--net=host \
--volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
--env=DISPLAY \
--name twr_jazzy_container \
twr:jazzy
```

Once you have the project built, you can continue with the [usage](../usage.md) examples.
