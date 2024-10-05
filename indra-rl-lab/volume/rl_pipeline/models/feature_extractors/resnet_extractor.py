import gymnasium as gym
from torch import nn

from rl_pipeline.models.feature_extractors import BaseFeaturesExtractor
from rl_pipeline.models.blocks import ResNetNN


class ResnetMLP(BaseFeaturesExtractor):

    def __init__(self,
                 observation_space: gym.spaces.Space,
                 features_dim: int
                 ):
        super().__init__(observation_space, features_dim)

        input_shape = observation_space.shape[0]
        self.resnets = nn.Sequential(
            nn.Linear(input_shape, features_dim),
            nn.LeakyReLU(),
            ResNetNN(features_dim),
            ResNetNN(features_dim),
            nn.Linear(features_dim, features_dim),
            nn.LeakyReLU(),
            )

    def forward(self, observations):
        return self.resnets(observations)
