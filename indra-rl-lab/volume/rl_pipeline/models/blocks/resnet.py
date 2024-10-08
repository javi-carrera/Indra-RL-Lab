# Project: Indra-RL-Lab
# File: environment.py
# Authors: Nicolás Rozado
# License: Apache 2.0 (refer to LICENSE file in the project root)

import torch as th
import torch.nn as nn


class ResNetNN(th.nn.Module):
    """Residual neural network"""
    def __init__(self, features_dim):
        super(ResNetNN, self).__init__()

        self.layer = nn.Sequential(
            nn.Linear(features_dim, features_dim),
            nn.LeakyReLU(),
            nn.Linear(features_dim, features_dim),
        )

    def forward(self, input_data):
        x = self.layer(input_data)
        x = th.add(x, input_data)
        x = nn.LeakyReLU()(x)
        return x
