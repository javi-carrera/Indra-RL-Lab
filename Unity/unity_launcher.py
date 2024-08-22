import subprocess
import time
import sys
import json


class UnityLauncher:


    @staticmethod
    def launch_unity_simulation(
        build_path: str,
        n_environments: int,
        headless_mode: bool,
        pause: bool,
        sample_time: float,
        time_scale: float
    ):

        processes = []
        
        # Start environments
        print(
            f"Starting {n_environments} environments with the following parameters:\n" \
            f"Build path: {build_path}\n" \
            f"Headless mode: {headless_mode}\n" \
            f"Pause: {pause}\n" \
            f"Sample time: {sample_time}\n" \
            f"Time scale: {time_scale}\n"
        )

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

            print("Stopping all environments...")

            # Sends SIGTERM on Unix, terminates process on Windows
            for p in processes:
                p.terminate()  

            # Wait for processes to exit after termination signal
            for p in processes:
                p.wait()
                
            print("All environments stopped.")



if __name__ == "__main__":
    

    config_file_path = "unity_config.json"

    with open(config_file_path) as f:
        config = json.load(f)

    unity_launcher = UnityLauncher()

    unity_launcher.launch_unity_simulation(
        build_path=config['build_path'],
        n_environments=config['n_environments'],
        headless_mode=config['headless_mode'],
        pause=config['pause'],
        sample_time=config['sample_time'],
        time_scale=config['time_scale']
    )
