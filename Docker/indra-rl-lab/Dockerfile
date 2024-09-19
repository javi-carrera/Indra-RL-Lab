# Project: Playground
# File: Dockerfile
# Authors: Javier Carrera
# License: Apache 2.0 (refer to LICENSE file in the project root)


FROM osrf/ros:humble-desktop


ENV USERNAME=user

# Create a non-root user
# ARG USERNAME=$USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc
    # && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc

# Install additional packages
RUN sudo apt update \
    && apt install -y pip \
    # Clean up
    && apt autoremove -y \
    && apt clean -y \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt .
RUN pip install --disable-pip-version-check --no-cache-dir -U -r requirements.txt \
    && rm requirements.txt


ENV PYTHONPATH="${PYTHONPATH}:/opt/ros/humble/local/lib/python3.10/dist-packages"
ENV PYTHONPATH="${PYTHONPATH}:/home/user/ros_ws/install/interfaces_pkg/local/lib/python3.10/dist-packages"
ENV PYTHONPATH="${PYTHONPATH}:/home/user/ros_ws/src/rl_pkg"

ENV PYTHONPATH="${PYTHONPATH}:/home/user/"

# Switch to the non-root user
USER $USERNAME

WORKDIR /home/$USERNAME

ENTRYPOINT ["tail", "-f", "/dev/null"]
# CMD ["/bin/bash"]
# CMD bash launch.bash




#### ROS2 COMMANDS ####

# Create a new package
# >> source /opt/ros/humble/setup.bash
# >> ros2 pkg create --build-type ament_python [package_name]
# >> ros2 pkg create --build-type ament_cmake [package_name]

# Build the package
# >> colcon build --symlink-install
# >> colcon build --symlink-install --packages-select [package_name]

# Run the package
# >> source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run [package_name] [executable_name]


#######################

# >> colcon build --symlink-install
# >> colcon build --symlink-install --packages-select playground_pkg
# >> colcon build --symlink-install --packages-ignore interfaces_pkg
# >> colcon build --packages-select interfaces_pkg

# >> source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run ros_tcp_endpoint default_server_endpoint
# >> source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0 -p ROS_TCP_PORT:=10000