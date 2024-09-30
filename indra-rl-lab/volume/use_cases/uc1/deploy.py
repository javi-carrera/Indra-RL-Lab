import yaml

from pathlib import Path

from rl_pipeline.algorithm_registry import ALGORITHMS, get_algorithm_kwargs
from use_cases.uc1 import UC1Environment


def deploy_uc1():

    # Configuration
    config_file_path = "config.yml"
    config = yaml.safe_load(open(config_file_path, 'r'))

    environment_config = config['environment']
    deployment_config = config['deployment']

    # Environment
    vec_env = UC1Environment.create_vectorized_environment(
        n_environments=environment_config['n_environments'],
        return_type="stable-baselines",
        monitor=True
    )
    
    # Algorithm
    algorithm = deployment_config['algorithm']
    algorithm_kwargs = get_algorithm_kwargs(
        env=vec_env,
        algorithm=algorithm
    )

    experiments_path = Path('experiments')
    environment_id = environment_config['id']
    experiment_name = deployment_config['experiment_name']
    log_dir = experiments_path / f"{environment_id}/{algorithm}/{experiment_name}"
    
    checkpoint_dir = log_dir / 'checkpoints'
    checkpoint = deployment_config['checkpoint']
    pretrained_model_path = checkpoint_dir / checkpoint
    
    model = ALGORITHMS[algorithm].load(pretrained_model_path, **algorithm_kwargs)

    # Deployment
    observations = vec_env.reset()
    reward_sum = 0

    while True:
        action, _ = model.predict(observations, deterministic=True)
        observations, rewards, dones, info = vec_env.step(action)
        # reward_sum += reward
        # env.render()

    # print(f"****** Total reward: {reward_sum}")