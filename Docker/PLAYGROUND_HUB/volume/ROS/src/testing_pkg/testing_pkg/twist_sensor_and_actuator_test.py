from typing import Tuple, Type

import gymnasium as gym
import numpy as np
import rclpy

from playground_pkg.single_agent_environment_node import SingleAgentEnvironmentNode
from playground_pkg.utils.pose_converter import PoseConverter
from playground_pkg.utils.vector_visualizer import VectorVisualizer
from interfaces_pkg.srv import TwistSensorAndActuatorTestEnvironmentAction, TwistSensorAndActuatorTestEnvironmentState, TwistSensorAndActuatorTestEnvironmentReset

import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R


class PoseSensorAndActuatorTestEnvironment(SingleAgentEnvironmentNode):

    def __init__(self):

        # ROS initialization
        SingleAgentEnvironmentNode.__init__(
            self,
            environment_name='twist_sensor_and_actuator_test_environment',
            action_service_msg_type=TwistSensorAndActuatorTestEnvironmentAction,
            state_service_msg_type=TwistSensorAndActuatorTestEnvironmentState,
            reset_service_msg_type=TwistSensorAndActuatorTestEnvironmentReset,
            sample_time=0.0
        )

        # Initialize the visualizer
        self.vector_visualizer = VectorVisualizer()



    def convert_action_to_request(self, action: np.ndarray) -> TwistSensorAndActuatorTestEnvironmentAction.Request:
        
        # action = np.array([vx, vy, vz, roll_rate, pitch_rate, yaw_rate])
        ros_linear_velocity = action[:3]
        ros_angular_velocity = action[3:]

        # Convert the position from ROS to Unity coordinate system
        unity_linear_velocity = PoseConverter.ros_to_unity_vector(ros_linear_velocity)

        # Convert the orientation from euler angles to quaternion
        unity_angular_velocity = PoseConverter.ros_to_unity_vector(ros_angular_velocity)

        # Convert the action to ROS request format
        self.action_request.action.target_twist.linear.x = unity_linear_velocity[0]
        self.action_request.action.target_twist.linear.y = unity_linear_velocity[1]
        self.action_request.action.target_twist.linear.z = unity_linear_velocity[2]

        self.action_request.action.target_twist.angular.x = unity_angular_velocity[0]
        self.action_request.action.target_twist.angular.y = unity_angular_velocity[1]
        self.action_request.action.target_twist.angular.z = unity_angular_velocity[2]

        return self.action_request
    

    def convert_response_to_state(self, response) -> dict:

        # Convert position from Unity to ROS coordinate system
        ros_linear_velocity = PoseConverter.ros_to_unity_vector(np.array([
            response.state.twist.linear.x,
            response.state.twist.linear.y,
            response.state.twist.linear.z,
        ]))

        # Convert orientation from Unity to ROS coordinate system
        ros_angular_velocity = PoseConverter.ros_to_unity_euler_vector(np.array([
            response.state.twist.angular.x,
            response.state.twist.angular.y,
            response.state.twist.angular.z,
        ]))

        # Convert the response to dict
        state = {
            'vx': ros_linear_velocity[0],
            'vy': ros_linear_velocity[1],
            'vz': ros_linear_velocity[2],
            'roll_rate': ros_angular_velocity[0],
            'pitch_rate': ros_angular_velocity[1],
            'yaw_rate': ros_angular_velocity[2],
        }

        return state
    

    def convert_reset_to_request(self) -> Type:
        return self.reset_request
    

    def render(self, state: np.ndarray):

        # print(
        #     f'Linear:\n'
        #     f'  vx: {state["vx"]}\n'
        #     f'  vy: {state["vy"]}\n'
        #     f'  vz: {state["vz"]}\n'
        #     f'Angular:\n'
        #     f'  roll_rate: {state["roll_rate"]}\n'
        #     f'  pitch_rate: {state["pitch_rate"]}\n'
        #     f'  yaw_rate: {state["yaw_rate"]}\n'
        # )

        # Visualize the linear velocity
        self.vector_visualizer.visualize(
            vectors=[self.state_response.state.twist.linear],
        )


    def observation(self, state: np.ndarray) -> np.ndarray:
        return state
    
    def reward(self, state: np.ndarray, action: np.ndarray = None) -> float:
        return 0.0
    
    def terminated(self, state: np.ndarray) -> bool:
        return False
    
    def truncated(self, state: np.ndarray) -> bool:
        return False

    def info(self, state: np.ndarray) -> dict:
        return {}


def main():

    rclpy.init()

    env = PoseSensorAndActuatorTestEnvironment()
    action = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    while True:
        state, reward, terminated, truncated, info = env.step(action)
        action = np.array([state['vx'], state['vy'], state['vz'], state['roll_rate'], state['pitch_rate'], state['yaw_rate']])
        env.render(state)

    env.close()


if __name__ == '__main__':
    main()
