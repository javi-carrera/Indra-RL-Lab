# Project: Playground
# File: launch_unity_simulation.py
# Authors: Javier Carrera, Rodrigo Sánchez
# License: Apache 2.0 (refer to LICENSE file in the project root)

import subprocess
import time
import yaml
import argparse


def launch_unity_simulation(
    n_environments: int,
    build_path: str,
    headless_mode: bool,
    pause: bool,
    sample_time: float,
    time_scale: float
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
            "--time-scale", str(time_scale),
        ]
        cmd += ["--headless"] if headless_mode else []

        print(f"Starting instance {i+1} on port {port} with environment ID {env_id}")

        proc = subprocess.Popen(cmd)
        processes.append(proc)

    try:

        # Keep the script running until all subprocesses are done
        while True:

            all_exited = all(p.poll() is not None for p in processes)

            if all_exited:
                break

            # Reduce CPU usage by limiting the check rate
            time.sleep(0.5)  

    except KeyboardInterrupt:

        print("\nStopping all environments...")

        # Sends SIGTERM on Unix, terminates process on Windows
        for p in processes:
            p.terminate()  

        # Wait for processes to exit after termination signal
        for p in processes:
            p.wait()
            
        print("All environments stopped.")



if __name__ == "__main__":

    config_file_path = "../Docker/indra-rl-lab/volume/config.yml"

    # Read the target machine from the command line
    parser = argparse.ArgumentParser()
    parser.add_argument('-m', '--machine', type=str)
    args = parser.parse_args()

    valid_machines = ['linux', 'windows']

    if args.machine not in valid_machines:
        raise ValueError(f"Invalid machine. Valid options are: {valid_machines}")

    # Load configuration file
    config = yaml.safe_load(open(config_file_path))

    n_environments = config['n_environments']

    build_path = f'builds/{args.machine}/{config["unity"]["build_name"]}'

    if args.machine == 'linux':
        build_path += '.x86_64' 
    elif args.machine == 'windows':
        build_path += '.exe'
    else:
        raise ValueError(f"Invalid machine: {args.machine}")

    headless_mode = config['unity']['headless_mode']
    pause = config['unity']['pause']
    sample_time = config['unity']['sample_time']
    time_scale = config['unity']['time_scale']


    print(
        f"\nStarting {n_environments} environments with the following parameters:\n" \
        f"Build path: {build_path}\n" \
        f"Headless mode: {headless_mode}\n" \
        f"Pause: {pause}\n" \
        f"Sample time: {sample_time}\n" \
        f"Time scale: {time_scale}\n"
    )

    # Start environments
    launch_unity_simulation(
        n_environments=n_environments,
        build_path=build_path,
        headless_mode=headless_mode,
        pause=pause,
        sample_time=sample_time,
        time_scale=time_scale
    )