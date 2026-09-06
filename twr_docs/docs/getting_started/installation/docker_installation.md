# Docker Installation

!!! warning

    GUI display requires X Window System on the host machine.


## Prerequisites

Follow the [instructions](https://docs.docker.com/engine/install/) from the official website to install Docker on your machine.

## 1. Clone repository

Clone the project from GitHub:

```shell
git clone https://github.com/AJedancov/twr.git
cd twr
```

## 2. Build Docker Image

Create a Docker image using the following command:

```shell
docker build -t twr:jazzy .
```

This will create an image according to the [Dockerfile](https://github.com/AJedancov/twr/blob/jazzy/Dockerfile) located at the root of the project. 

Add the docker to the host X server to enable GUI rendering:

```shell
sudo xhost +local:docker
```

## 3. Run Docker Container

And finally, you can run the container:

```shell
docker run \
    --interactive \
    --tty \
    --rm \
    --net=host \
    --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
    --env=DISPLAY \
    --name=twr_jazzy_container \
    twr:jazzy
```

??? note "Flags description"
 
    This command includes the following options:  
    `--interactive` - keep terminal open.  
    `--tty` - allocate a pseudo-TTY.  
    `--rm` - automatically remove the container when it exits.  
    `--net=host` - connect to the host network.  
    `--volume` - link the volume mount. This is necessary for correct GUI rendering.  

Once the Docker container is running, continue with [usage examples](../usage.md).
