from stable_baselines3.common.env_checker import check_env
from stable_baselines3 import PPO, DDPG
from gymnasium.vector import AsyncVectorEnv
import json

from stable_baselines3.common.vec_env import SubprocVecEnv

from examples_pkg.environments.autonomous_navigation_example_environment import AutonomousNavigationExampleEnvironment, create_environment

def main():

    # Load the configuration file
    config_file_path = "ros_config.json"
    with open(config_file_path, "r") as f:
        config = json.load(f)


    # Create the vectorized environment
    vec_env = SubprocVecEnv(
        [lambda env_id=i: create_environment(env_id) for i in range(config["n_environments"])],
        start_method='spawn'
    )
    
    vec_env.reset()

    n_timesteps = 5e6

    model = DDPG(
        policy="MlpPolicy",
        env=vec_env,
        verbose=1,
        tensorboard_log="./tensorboard_logs/",
    )

    # Train the agent
    model.learn(total_timesteps=int(n_timesteps))


if __name__ == "__main__":
    main()