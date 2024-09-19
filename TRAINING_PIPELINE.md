# Training Pipeline

This repository provides a framework for performing Reinforcement Learning (RL) experiments using [Stable Baselines 3](https://stable-baselines3.readthedocs.io/) and logging results to [Weights & Biases](https://wandb.ai/). It is designed for students to easily run experiments, upload their own model architectures, and analyze results.

## Table of contents
- [🔧 Installation](#installation)
- [📁 Project structure](#project-structure)
- [📖 Getting Started](#getting-started)
    - [Modifying Configurations](#modifying-configurations)
    - [Adding Custom Models](#adding-custom-models)
    - [Running Training](#running-training)
    - [Evaluating Agents](#evaluating-agents)
- [Logging and Monitoring](#logging-and-monitoring)
- [Contributing](#contributing)
- [License](#license)


## Installation

**1.** Clone the Indra-RL-Lab repository.

```git
git clone https://github.com/javi-carrera/Indra-RL-Lab.git
cd your-repo-name
```

**2.** Follow the steps in [the main README](README.md) to have the environment and project running.

**3.** The container will handle requirements and dependencies installation automatically.

## Repository Structure

The only files you should be modifying here are the *config files* and the custom models to add your own torch architectures.

```css
├── configs/
│   └── algorithm_config.yaml
├── experiments/
│   └── [Environment ID]/
│       └── [Algorithm]/
│           └── [Experiment Name]/
├── rl_pipeline/
│   ├── models/
│   │   ├── [Custom Blocks]
│   │   └── [Custom Feature Extractors]
│   ├── run/
│   │   └── rl_trainer.py
│   └── utils/
│       └── [Utility Scripts]
├── play.py
└── train.py
```
- **configs/**: Contains YAML configuration files for experiments. Here everything realted to the trainig is set up, you'll find specific configurations for each algorithm including all the hyperparameters. 
- **experiments/**: Stores experiment data and results locally.
- **rl_pipeline/**:
    - **models/**: Directory where students can add their custom model architectures and blocks.
    - **run/**:
        - ```rl_trainer.py```: The main trainer class.
        - ```train.py```: Script to initiate training.
        - ```play.py```: Script to run a trained agent.
    - **utils/**: Utility scripts for environment setup and other functionalities.

## Getting Started

### Modifying Configurations

To set up an experiment, modify one of the YAML configuration files in the ```configs/``` directory or create a new one. Here you can add your preferred hiperparameters architecture and specify a path to a trained model for preloading it.

**Example configuration file**: ```configs/base_ppo_config.yaml```

```yaml
experiment:
  name: 'BipedalWalker-Benchmark'

environment:
  id: 'BipedalWalker-v3'
  env_config: 'None'
  render_mode: 'rgb_array'
  monitor: true
  video_wrapper: true
  video_trigger: 5000
  video_length: 500

training:
  algorithm: 'ppo'
  pretrained_model: 'None'
  use_wandb: true
  algorithm_parameters:
    policy: 'MlpPolicy'
    learning_rate: 0.0003
    n_steps: 8192
    batch_size: 512
    n_epochs: 5
    gamma: 0.99
    gae_lambda: 0.95
    clip_range: 0.2
    ent_coef: 0.0
    vf_coef: 0.5
    max_grad_norm: 0.5

  architecture:
    net_arch: {'pi': [128, 128], 'vf': [128, 128]}
    features_extractor_class: 'ResnetMLP'
    features_extractor_kwargs:
      features_dim: 128
    activation_fn: 'ReLU'
    share_features_extractor: false

  training:
    eval:
      seed: 5
      num_episodes: 5
      num_evals: 15
    total_timesteps: 500000
    device: 'cuda'
    log_points: 10
    verbose: 2

evaluation:
  num_episodes: 1

play:
  experiment: 'experiment_date'
  pretrained_model: 'model'

```
### Adding Custom Models

To use a custom architecture, add your PyTorch model class to the ```rl_pipeline/models/feature_extractors``` directory. Your model should inherit from the base extractor class provided in ```rl_pipeline/models/feature_extractors/base_extractor```.

##### Example

```python
# rl_pipeline/models/custom_cnn.py

import torch
from rl_pipeline.models.feature_extractors.base_extractor import BaseFeaturesExtractor

class CustomCNN(BaseFeaturesExtractor):
    def __init__(self, observation_space, features_dim=256):
        super(CustomCNN, self).__init__(observation_space, features_dim)
        # Define your custom layers here

    def forward(self, observations):
        # Implement forward pass
        return features
```
Update the configuration to use your custom model:
```yaml
architecture:
  features_extractor_class: 'CustomCNN'
  features_extractor_kwargs:
    features_dim: 256
```
### Running Training

To run an experiment we will follow the steps mentioned in the main [readme](REAMDE.md):

**1.** Define the simulation parameters:

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

**2.** Launch the Unity simulation:

```
launch_unity_simulation.bat
```

**3.** In the Visual Studio Code attached to the running container, open two new terminals and run the following commands in each one of them:

```bash
bash launch_ros_tcp_endpoint.bash
```

```bash
bash launch_node.bash
```

The `launch_node.bash` file will lauch the package and node specified in the configuration, executing the training logic.

#### Other Environments

Alternatively you can also use this pipeline to run experiments on any other gymnasium environment running the ```train_example.py``` script with the desired configuration file:
```python
python train_example.py --config configs/base_ppo_config.yaml
```

## Logging and Monitoring
The framework integrates with Weights & Biases for experiment tracking and logging.

- **Enabling W&B Logging**: 
1. Set ```use_wandb: true``` in the ```training``` section of your configuration.
2. Run ```wandb login``` in a terminal within the dev container and enter your [api key](https://wandb.ai/authorize). **This step only needs to be done once**, after that the api key is appended to an ntrc file and saved for later runs to be logged automatically.
- **W&B logs access**: Ensure you're logged in to W&B and you should have access to all the runs online within a project named ```sb3``` (if needed, the project name can be changed in the ```RLTrainer``` class initialization).

**Local Logging**: All experiment data, including models and logs, are saved in the experiments/ directory following the structure:
```css
experiments/
└── [Environment ID]/
    └── [Algorithm]/
        └── [Experiment Name]/
            ├── videos/
            ├── model.zip
```