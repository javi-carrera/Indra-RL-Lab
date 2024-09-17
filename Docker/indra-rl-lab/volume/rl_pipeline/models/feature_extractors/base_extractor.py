import gymnasium as gym
import torch


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
