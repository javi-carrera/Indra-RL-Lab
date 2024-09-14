import gymnasium as gym
from  typing import Dict, Callable


def register_custom_env(env_id: str, env_creator) -> None:
    """Registers the specified environment with Gymnasium if it is defined in the registry and not already registered."""
    try:
        # Check if the environment is already registered
        gym.spec(env_id)
        print(f"Environment '{env_id}' is already registered.")
    except gym.error.UnregisteredEnv:
        # Register the environment if it's defined in the registry and not already registered
        gym.register(
            id=env_id,
            entry_point=env_creator
        )
        print(f"Environment '{env_id}' registered successfully.")

