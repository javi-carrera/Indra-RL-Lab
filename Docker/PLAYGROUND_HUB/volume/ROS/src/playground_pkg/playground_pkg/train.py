import numpy as np
import rclpy
from playground_pkg.environment import Environment

def main():

    rclpy.init()

    env = Environment()

    action = np.array([1.0, 0.0, 0.0])

    while True:

        state, reward, terminated, truncated, info = env.step(action)

        if terminated or truncated:
            state, _ = env.reset()
            print('Reset')

        print(f'State: {state}')

    env.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()