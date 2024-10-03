import yaml

from pathlib import Path

from rl_pipeline.algorithm_registry import ALGORITHMS, get_algorithm_kwargs
from use_cases.uc3 import UC3Environment


def deploy_uc3():

    # Configuration
    config_file_path = "config.yml"
    config = yaml.safe_load(open(config_file_path, 'r'))
    environment_config = config['environment']
    deployment_config = config['deployment']

    experiments_path = Path('experiments')
    environment_id = environment_config['id']
    experiment_name = deployment_config['experiment_name']
    log_dir = experiments_path / f"{environment_id}/{experiment_name}"
    experiment_training_config = yaml.safe_load(open(log_dir / 'training_config.yml', 'r'))

    # Environment
    vec_env = UC3Environment.create_vectorized_environment(
        n_environments=environment_config['n_environments'],
        return_type="stable-baselines",
        monitor=True
    )
    
    # Algorithm
    algorithm = experiment_training_config['algorithm']
    algorithm_kwargs = get_algorithm_kwargs(
        env=vec_env,
        algorithm=algorithm
    )
    
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