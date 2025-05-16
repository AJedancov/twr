
ARG ROSDISTRO=jazzy

# Base image
FROM ros:$ROSDISTRO

ARG USERNAME=dev-user
ARG GROUPNAME=dev-group
ARG UID=1001
ARG GID=$UID
ARG WORKDIR=/home/$USERNAME/twr

ENV TWR_WS=$WORKDIR

# Create a non-root user
RUN groupadd \
    --gid $GID $GROUPNAME \
    && useradd \
    --create-home \ 
    --shell /bin/bash \
    --gid $GID \
    --uid $UID \
    $USERNAME \
    && mkdir /home/$USERNAME/.config \
    && chown $UID:$GID /home/$USERNAME/.config

# All subsequent commands will be executed in $WORKDIR as $USERNAME
USER $USERNAME
WORKDIR $WORKDIR 

# Copy entire workspace (twr) to WORKDIR
# Except for the files specified in .dockerignore
COPY --chown=$UID:$GID . $WORKDIR

# Install all dependencies
USER root
ARG ROSDISTRO
RUN apt-get update \
    && rosdep update \
    --rosdistro $ROSDISTRO \ 
    && rosdep install -y \
    --from-paths $WORKDIR \
    --rosdistro $ROSDISTRO \
    --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# Provide entrypoint for Docker container
RUN chmod +x $WORKDIR/scripts/docker/twr_entrypoint.bash
ENTRYPOINT [ "/bin/bash", "scripts/docker/twr_entrypoint.bash" ]
CMD ["bash"]

USER $USERNAME