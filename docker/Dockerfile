# ROS distribution to use
ARG ROS_DISTRO=foxy

########################################
# Base Image for TurtleBot3 Simulation #
########################################
FROM px4io/px4-dev-ros2-{ROS_DISTRO} AS dev

# Update git to fix git version mismatch with bind mount repos
RUN sudo apt-get install software-properties-common
RUN sudo apt-add-repository ppa:git-core/ppa \
 && sudo apt-get update \
 && sudo apt-get install git

# Dev container arguments
ARG USERNAME=devuser
ARG UID=1000
ARG GID=${UID}

# Install extra tools for development
# RUN apt-get update && apt-get install -y --no-install-recommends \
#  gdb gdbserver nano

# Create new user and home directory
RUN groupadd --gid $GID $USERNAME \
 && useradd --uid ${GID} --gid ${UID} --create-home ${USERNAME} \
 && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME} \
 && mkdir -p /home/${USERNAME} \
 && chown -R ${UID}:${GID} /home/${USERNAME}

# Set the ownership of the overlay workspace to the new user
RUN chown -R ${UID}:${GID} /home/ws

# Set the user and source entrypoint in the user's .bashrc file
USER ${USERNAME}
# RUN echo "source /entrypoint.sh" >> /home/${USERNAME}/.bashrc