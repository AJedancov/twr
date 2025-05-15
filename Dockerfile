
ARG ROSDISTRO=jazzy

# Base image
FROM ros:$ROSDISTRO

# ARG USERNAME=dev-user
# ARG GROUPNAME=dev-group
# ARG UID=1001
# ARG GID=$UID
ARG WORKDIR=/twr

# Create a non-root user
# RUN groupadd \
#     --gid $GID $GROUPNAME \
#     && useradd \ 
#     --shell /bin/bash \
#     --gid $GID \
#     --uid $UID $USERNAME

# All subsequent commands will be executed in $WORKDIR
WORKDIR $WORKDIR 

# Copy entire workspace (twr) to WORKDIR
# Except for the files specified in .dockerignore
# COPY --chown=$USERNAME:$GROUPNAME . $WORKDIR
COPY . $WORKDIR

# Install all dependencies
ARG ROSDISTRO
RUN apt-get update \
    && apt-get install -y \
    && rosdep update \
    --rosdistro $ROSDISTRO \ 
    && rosdep install -y \
    --from-paths $WORKDIR \
    --rosdistro $ROSDISTRO \
    --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# USER $USERNAME

# Provide entrypoint for Docker container
RUN chmod +x $WORKDIR/scripts/docker/twr_entrypoint.bash
ENTRYPOINT [ "/bin/bash", "scripts/docker/twr_entrypoint.bash" ]
CMD ["bash"]