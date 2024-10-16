# Project: Indra-RL-Lab
# File: deploy.py
# Authors: Javier Carrera
# License: Apache 2.0 (refer to LICENSE file in the project root)

import yaml

from rl_pipeline import RLDeployer
from use_cases.uc2 import UC2Environment, UC2ObservationWrapper, UC2RewardWrapper

from rl_pkg.visualizers.esco_visualizer import MainApp

def deploy_uc2():
    app = MainApp()
    app.start_plotting()

    # Configuration
    config_file_path = "config.yml"
    config = yaml.safe_load(open(config_file_path, 'r'))

    # Environment
    vec_env = UC2Environment.create_vectorized_environment(
        n_environments=config['environment']['n_environments'],
        return_type="stable-baselines",
        monitor=True,
        wrappers=[
            UC2ObservationWrapper,
            UC2RewardWrapper
        ]
    )
    
    # Deployer
    deployer = RLDeployer(
        env=vec_env,
        environment_config=config['environment'],
        deployment_config=config['deployment'],
    )
    # deployer.deploy()

    observations = deployer.env.reset()

    try:
        while True:
            action, _ = deployer.model.predict(observations, deterministic=True)
            observations, reward, dones, info = deployer.env.step(action)
            app.send_data_to_plot(None, reward, dones)
            # # action = np.random.uniform(-1.0, 1.0, size=3)
            # linear_velocity = np.random.normal(0, 2.0)
            # angular_velocity = np.random.normal(0, 2.0)
            # fire = np.random.choice([0, 1])
            # action = np.array([linear_velocity, angular_velocity, fire, 0.0])

            # if terminated or truncated:
            #     deployer.env.reset()
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
    finally:
        app.stop_plotting()
        deployer.env.close()