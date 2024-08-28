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
| Unity Editor                                    | [[download link]](https://unity.com/download)                      | 2022.3.36f1    |
| Docker Desktop                                  | [[download link]](https://www.docker.com/products/docker-desktop/) | 4.33.1         |
| Windows X Server (for Docker GUI visualization) | [[download link]](https://sourceforge.net/projects/vcxsrv/)        | 64.1.20.14.0   |


## Getting Started

### Installation

**1.** Clone the repository.

**2.** Start the [docker-compose.yml](./Docker/docker-compose.yml) file.

**3.** Build the ROS workspace in the `playground_hub` container by running:

```bash
bash build.bash
```

### Deployment

**1.** Run the Unity simulation. This can be done in the Unity Editor (for developement) or running the build (for deployment).
    
-  Running the scene from the Unity Editor.
    1. Open the Unity Project in [Unity/Playground](./Unity/Playground/).
    2. In the Unity Editor, open and play the `AutonomousNavigationExample` scene in [Unity/Playground/Assets/Playground/Scenes](./Unity/Playground/Assets/Playground/Scenes)

- Running the build.
    1. In the [Unity](./Unity/) directory run:
```
launch_unity_simulation.bat
```

**3.** Run Windows X Server for Docker GUI visualization (Optional)

**4.** Attach two terminals to the `playground_hub` container, and run the following commands in each one of them:
```bash
bash launch_ros_tcp_endpoint.bash
```
```bash
bash launch_node.bash
```

### Developement

[developement]

## Project structure

[project structure]





