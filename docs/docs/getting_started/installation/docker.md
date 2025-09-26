# Docker

!!! warning
    Currently, the GUI is displayed when the X Window System is in use. 


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


## Run Docker container

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

??? note "Description of options" 
    This command includes the following options:  
    `--interactive` - keep terminal open.  
    `--tty` - allocate a pseudo-TTY.  
    `--rm` - automatically remove the container when it exits.  
    `--net=host` - connect to the host network.  
    `--volume` - link the volume mount. This is necessary for correct GUI rendering.  

Once you have the project built, you can continue with the [usage](../usage.md) examples.
