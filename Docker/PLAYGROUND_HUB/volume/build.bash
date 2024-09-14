cd /home/$USERNAME/ros_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# colcon build --packages-select interfaces_pkg
# colcon build --symlink-install --packages-ignore interfaces_pkg