# Playground

[description]

## Table of contents
- [🏠 About](#about)
- [✨ Features](#features)
- [📋 Prerequisites](#prerequisites)
- [📖 Getting Started](#getting-started)
    - [🔧 Installation](#installation)
    - [🚀 Deployment](#deployment)
- [📁 Project structure](#project-structure)

## About
[about]


## Features
[features]


## Prerequisites

| Software                                        | Download link                                                      | Tested Version |
|-------------------------------------------------|--------------------------------------------------------------------|----------------|
| Docker Desktop                                  | [[download link]](https://www.docker.com/products/docker-desktop/) | 4.33.1         |
| Visual Studio Code                              | [[download link]](https://code.visualstudio.com/download)          | -              |
| Windows X Server (for Docker GUI visualization) | [[download link]](https://sourceforge.net/projects/vcxsrv/)        | 64.1.20.14.0   |


## Getting Started

The framework consists of two primary components:

- **Unity Simulation:** Handles the physics simulation and manages sensors, actuators, agents, and environment behaviors.
- **ROS2 Python Node:** Runs within a Docker container, communicating with the Unity simulation through a client-server setup.


#### Unity Simulation
The simulation is run Unity standalone build because of deployment purposes as it:
- Supports headless mode (no graphical output).
- Enables parallel execution of multiple environment instances, enhancing training efficiency and scalability.

#### ROS2 Python Node

The ROS2 node operates as a client that interfaces with the Unity simulation server, managing the exchange data between the simulation and the algorithm. This behavior is framed as a conventional reinforcement learning gym/gymnasium environment by defining the `step`, `reset`, `render` and `close` methods.
- `step(action)`: Sends an action to the Unity simulation and returns the new state, reward, terminated and truncated flags.
- `reset()`: Reinitializes the environment to a starting state for a new episode.
- `render()`: (Optional) Provides a method for visualizing the state of the environment, useful during development and debugging.
- `close()`: Properly shuts down the connection and cleans up resources to ensure a graceful termination of the session.

### Installation

**1.** Clone the repository.

**2.** Install the [Docker](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker) and [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extensions.

**3.** Navigate to the [docker-compose.yml](Docker/docker-compose.yml) file and right-click 'Compose Up' to start the container (Ensure Docker Desktop is running).

![](docs/images/docker_compose_up.png)

<!-- *NOTE: If your PC lacks a dedicated Nvidia graphics card, use the [docker-compose-no-gpu.yml](Docker/docker-compose-no-gpu.yml) file instead. -->


**3.** Attach a Visual Studio Code to the running container by right-clicking on the running container in the Docker extension tab, and selecting 'Attach Visual Studio Code'.

![](docs/images/docker_attach_vscode.png)

**3.** Once attached to the running container, open a new terminal in the directory `/home` and build the ROS workspace by running:

```bash
bash build.bash
```

Expected output:

```output
Building ROS project...
Command succeeded: bash -c 'cd /home/ros && source /opt/ros/humble/setup.bash && colcon build --packages-select interfaces_pkg'
Starting >>> interfaces_pkg
Finished <<< interfaces_pkg [10.9s]

Summary: 1 package finished [11.4s]

Command succeeded: bash -c 'cd /home/ros && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-ignore interfaces_pkg'
Starting >>> examples_pkg
Starting >>> playground_pkg
Starting >>> ros_tcp_endpoint
Finished <<< examples_pkg [3.34s]
Finished <<< playground_pkg [3.50s]
Finished <<< ros_tcp_endpoint [3.66s]

Summary: 3 packages finished [4.09s]

ROS project built successfully!
```

### Deployment

**1.** Launch the Unity simulation by openning a terminal in the [Unity](Unity/) directory and run:
```
launch_unity_simulation.bat
```

**2.** (Optional) Run Windows X Server for Docker GUI visualization.

**3.** In the Visual Studio Code attached to the running container, open two new terminals and run the following commands in each one of them:

```bash
bash launch_ros_tcp_endpoint.bash
```

```bash
bash train.bash
```

Expected output in the first terminal:

```output
Starting 1 ROS TCP endpoints...

Starting ROS instance 0 on TCP port 10000

[INFO] [timestamp] [UnityEndpoint]: Starting server on 0.0.0.0:10000
[INFO] [timestamp] [UnityEndpoint]: Connection from 172.20.0.1
[INFO] [timestamp] [UnityEndpoint]: RegisterUnityService(...) OK
[INFO] [timestamp] [UnityEndpoint]: RegisterUnityService(...) OK
```

Once the setup is done, the vehicle should be seen executing random actions within the scene.


### Developement

**1.** Customize environment behavior and training logic.

To customize the environment definition, and set up training for the reinforcement learning algorithms in the `AutonomousNavigationExample`, the files [autonomous_navigation_example_environment.py](Docker/PLAYGROUND_HUB/volume/ROS/src/examples_pkg/examples_pkg/environments/autonomous_navigation_example_environment.py) and [train.py](Docker/PLAYGROUND_HUB/volume/ROS/src/examples_pkg/examples_pkg/train.py) must be modified.

- In `autonomous_navigation_example_environment.py`, the `observation()`, `reward()`, `terminated()`, `truncated()`, `info()` and `render()` modified to tailor the environment behavior.

- The `train.py` file must define the training logic. Adjust this file to align with your specific training requirements.

**2.** Define the simulation parameters:

In order to launch the training, the [config.yml](Docker/PLAYGROUND_HUB/volume/config.yml) file must be modified, specifying the `package` and the `node` fields accordingly:

```
ros:
  package_name: "examples_pkg"
  node_name: "train"
```

When running the environment as a Unity standalone build, other parameters such as the number of parallel environments, the time scale of the simulation, the pause and the headless mode flags can be modified in this config file:

```
n_environments: 1

ros:
  package_name: "examples_pkg"
  node_name: "train"

unity:
  build_path: "build/Playground.exe"
  headless_mode: false
  pause: false
  sample_time: 0.0
  time_scale: 1.0
```

**2.** Launch the Unity simulation to run an instance or parallel instances of the environment.

**3.** In the Visual Studio Code attached to the running container, open two new terminals and run the following commands in each one of them:

```bash
bash launch_ros_tcp_endpoint.bash
```

```bash
bash train.bash
```

The `train.bash` file will lauch the package and node specified in the configuration, executing the training logic.

## Project structure

[project structure]

## Training Pipeline

For detailed instructions on how to set up and run the training pipeline, please refer to the [Training Pipeline Guide](./TRAINING_PIPELINE.md).

