ARG ROS_DISTRO=jazzy
ARG BASE_IMAGE=ros:$ROS_DISTRO


FROM $BASE_IMAGE AS cashe

ARG MANIFESTS_DIR=/manifests/
COPY --parents ./twr_**/package.xml $MANIFESTS_DIR

RUN apt-get update \
    && rosdep update \
    --rosdistro $ROS_DISTRO \ 
    && rosdep install -y \
    --from-paths $MANIFESTS_DIR \
    --rosdistro $ROS_DISTRO \
    --ignore-src \
    && rm -rf /var/lib/apt/lists/*


FROM cashe AS build

ARG USERNAME=dev-user
ARG GROUPNAME=dev-group
ARG UID=1001
ARG GID=$UID

RUN groupadd \
    --gid $GID $GROUPNAME \
    && useradd \
    --create-home \ 
    --shell /bin/bash \
    --gid $GID \
    --uid $UID \
    $USERNAME

# Set the password to "twr" to use sudo commands.
RUN echo "$USERNAME:twr" | chpasswd && adduser $USERNAME sudo

ARG TWR_WS=/home/$USERNAME/twr
WORKDIR $TWR_WS
RUN chown $UID:$GID $TWR_WS
COPY --chown=$UID:$GID . $TWR_WS

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
      --symlink-install

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "source $TWR_WS/install/setup.bash" >> /home/$USERNAME/.bashrc

USER $USERNAME
CMD ["bash"]