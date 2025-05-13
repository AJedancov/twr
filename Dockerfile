# Base image
FROM ros:jazzy

# Install all dependencies
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-gz-ros2-control \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-slam-toolbox \
    && rm -rf /var/lib/apt/lists/* 

# Create a non-root user
ARG USERNAME=dev-user
ARG GROUPNAME=dev-group
ARG UID=1001
ARG GID=$UID

RUN groupadd --gid $GID $GROUPNAME \
    && useradd -s /bin/bash -g $GID -u $UID $USERNAME

ARG WORKDIR=/twr

# All subsequent commands will be in $WORKDIR as $USERNAME
WORKDIR $WORKDIR 

# Copy entire workspace (twr) to WORKDIR
# Except for the files specified in .dockerignore
COPY . .

# Provide entrypoint for Docker container
RUN chmod +x scripts/docker/twr_entrypoint.bash
ENTRYPOINT [ "/bin/bash", "scripts/docker/twr_entrypoint.bash" ]
CMD ["bash"]

# USER $USERNAME