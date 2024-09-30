# Project: Playground
# File: launch_unity_simulation.py
# Authors: Javier Carrera, Rodrigo Sánchez
# License: Apache 2.0 (refer to LICENSE file in the project root)

import platform
import subprocess
import time
import yaml


def launch_unity_simulation(
    n_environments: int,
    build_path: str,
    headless_mode: bool,
    pause: bool,
    sample_time: float,
    fixed_delta_time: float,
    fixed_updates_per_step: int,
):

    processes = []

    for i in range(n_environments):

        port = 10000 + i
        env_id = i
        
        cmd = [
            build_path,
            "--ros-port", str(port),
            "--environment-id", str(env_id),
            "--pause", str(pause),
            "--sample-time", str(sample_time),
            "--fixed-delta-time", str(fixed_delta_time),
            "--fixed-updates-per-step", str(fixed_updates_per_step),
        ]
        cmd += ["--headless"] if headless_mode else []

        print(f"Starting instance {i+1} on port {port} with environment ID {env_id}")

        proc = subprocess.Popen(cmd)
        processes.append(proc)

    try:
        while True:
            all_exited = all(p.poll() is not None for p in processes)
            if all_exited:
                break
            time.sleep(0.5)  

    except KeyboardInterrupt:

        print("\nStopping all environments...")

        for p in processes:
            p.terminate()

        for p in processes:
            p.wait()
            
        print("All environments stopped.")


if __name__ == "__main__":

    config_file_path = "../indra-rl-lab/volume/config.yml"
    config = yaml.safe_load(open(config_file_path))
    environment_config = config['environment']

    n_environments = environment_config['n_environments']
    use_case = environment_config['id']
    headless_mode = environment_config['unity']['headless_mode']
    pause = environment_config['unity']['pause']
    sample_time = environment_config['unity']['sample_time']
    fixed_delta_time = environment_config['unity']['fixed_delta_time']
    fixed_updates_per_step = environment_config['unity']['fixed_updates_per_step']

    available_systems_extensions = {
        "windows": ".exe",
        "linux": ""
    }

    system = platform.system().lower()
    extension = available_systems_extensions.get(system)
    if extension is None:
        raise Exception("Unsupported OS")
    
    build_path = f'./builds/{system}/{use_case}/{use_case}'

    print(
        f"\nStarting {n_environments} environments with the following parameters:\n" \
        f"Build path: {build_path}\n" \
        f"Headless mode: {headless_mode}\n" \
        f"Pause: {pause}\n" \
        f"Sample time: {sample_time}\n" \
        f"Fixed delta time: {fixed_delta_time}\n" \
        f"Fixed updates per step: {fixed_updates_per_step}\n"
        f'Expected time scale: {sample_time / fixed_delta_time / fixed_updates_per_step}\n'
    )

    launch_unity_simulation(
        n_environments=n_environments,
        build_path=build_path,
        headless_mode=headless_mode,
        pause=pause,
        sample_time=sample_time,
        fixed_delta_time=fixed_delta_time,
        fixed_updates_per_step=fixed_updates_per_step,
    )