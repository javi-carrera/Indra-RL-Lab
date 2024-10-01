import numpy as np
from stable_baselines3 import PPO

# Exponential decay learning rate scheduler
def exponential_schedule(initial_value, final_value):
    """
    Exponential learning rate schedule.
    
    :param initial_value: (float) Initial learning rate.
    :param decay_rate: (float) The decay rate. Smaller values decay slower.
    :return: (function) A function that computes the current learning rate given the progress.
    """
    def func(progress_remaining):
        """
        Progress will decrease from 1 (beginning) to 0 (end).
        
        :param progress_remaining: (float) Current progress remaining (from 1 to 0).
        :return: (float) Current learning rate.
        """
        decay_rate = -np.log(final_value / initial_value)
        return initial_value * np.exp(-decay_rate * (1 - progress_remaining))
    
    return func


# Cosine annealing learning rate scheduler
def cosine_schedule(initial_value):
    """
    Cosine learning rate schedule.
    
    :param initial_value: (float) Initial learning rate.
    :return: (function) A function that computes the current learning rate given the progress.
    """
    def func(progress_remaining):
        """
        Progress will decrease from 1 (beginning) to 0 (end).
        
        :param progress_remaining: (float) Current progress remaining (from 1 to 0).
        :return: (float) Current learning rate.
        """
        return initial_value * (0.5 * (1 + np.cos(np.pi * (1 - progress_remaining))))
    
    return func

