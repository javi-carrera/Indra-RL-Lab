from stable_baselines3.common.env_checker import check_env
from stable_baselines3 import PPO
from gymnasium.vector import AsyncVectorEnv
import json

from stable_baselines3.common.vec_env import SubprocVecEnv

from examples_pkg.autonomous_navigation_example import AutonomousNavigationExampleEnvironment, create_environment

def main():

    # Load the configuration file
    config_file_path = "ros_config.json"
    with open(config_file_path, "r") as f:
        config = json.load(f)


    # Training parameters
    normalize = True
    n_timesteps = 5e6
    policy = 'MlpPolicy'
    n_steps = 2048
    batch_size = 64
    gae_lambda = 0.95
    gamma = 0.999
    n_epochs = 10
    ent_coef = 0.0
    learning_rate = 3e-4
    clip_range = 0.18


    # Create the vectorized environment
    vec_env = SubprocVecEnv(
        [lambda env_id=i: create_environment(env_id) for i in range(config["n_environments"])],
        start_method='spawn'
    )
    
    vec_env.reset()

    # Create the agent
    model = PPO(
        policy,
        vec_env,
        verbose=1,
        learning_rate=learning_rate,
        n_steps=n_steps,
        batch_size=batch_size,
        n_epochs=n_epochs,
        gamma=gamma,
        gae_lambda=gae_lambda,
        clip_range=clip_range,
        ent_coef=ent_coef,
        normalize_advantage=normalize,
    )

    # Train the agent
    model.learn(total_timesteps=int(n_timesteps))


if __name__ == "__main__":
    main()