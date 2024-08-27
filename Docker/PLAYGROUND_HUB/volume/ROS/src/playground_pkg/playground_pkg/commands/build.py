# Project: Playground
# File: build.py
# Authors: Javier Carrera
# License: Apache 2.0 (refer to LICENSE file in the project root)


import subprocess


def run_command(command):

    try:
        # Run the command
        process = subprocess.run(command, shell=True, check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        print(f"Command succeeded: {command}")
        print(process.stdout)

    except subprocess.CalledProcessError as e:
        print(f"Error running command: {command}")
        print(e.stderr)
        raise


def build_ros_project():

    # Change directory to the ROS workspace
    cd_command = "cd /home/ros"
    # Source ROS setup
    source_command = "source /opt/ros/humble/setup.bash"
    # Execute colcon build for the specified package
    build_interfaces_command = "colcon build --packages-select interfaces_pkg"
    # Execute colcon build for the rest of the packages with symlink
    build_symlink_packages_command = "colcon build --symlink-install --packages-ignore interfaces_pkg"
    
    # Combining the source command with each colcon build command
    build_interfaces_command_full = f"bash -c '{cd_command} && {source_command} && {build_interfaces_command}'"
    build_symlink_packages_command_full = f"bash -c '{cd_command} && {source_command} && {build_symlink_packages_command}'"

    # Run the commands
    print("Building ROS project...")
    run_command(build_interfaces_command_full)
    run_command(build_symlink_packages_command_full)
    print("ROS project built successfully!")

if __name__ == "__main__":
    build_ros_project()
