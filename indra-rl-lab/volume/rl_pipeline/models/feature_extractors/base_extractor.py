# Project: Indra-RL-Lab
# File: base_extractor.py
# Authors: Nicolás Rozado
# License: Apache 2.0 (refer to LICENSE file in the project root)

import gymnasium as gym
import torch


## Fom Stable Baselines 3, expected to be modified eventually

class BaseFeaturesExtractor(torch.nn.Module):
    """
    Base class that represents a feature extractor.

    :param observation_space: observation space of the environment
    :type observation_space: gym.Space

    :param features_dim: Number of features extracted [embedding size]
    :type features_dim: int

    :raises ValueError: If features_dim is not strictly positive.
    """

    def __init__(self, observation_space: gym.Space, features_dim: int) -> None:
        super().__init__()
        if features_dim < 1:
            raise ValueError("features_dim must be positive, got {}".format(features_dim))

        self._observation_space = observation_space
        self._features_dim = features_dim

    @property
    def features_dim(self) -> int:
        return self._features_dim
