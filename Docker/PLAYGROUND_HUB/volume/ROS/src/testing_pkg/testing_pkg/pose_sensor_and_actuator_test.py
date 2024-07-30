from typing import Tuple, Type

import gymnasium as gym
import numpy as np
import rclpy

from playground_pkg.single_agent_environment_node import SingleAgentEnvironmentNode
from playground_pkg.utils.pose_converter import PoseConverter
from interfaces_pkg.srv import PoseSensorAndActuatorTestEnvironmentStep

import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R


class PoseSensorAndActuatorTestEnvironment(SingleAgentEnvironmentNode):

    def __init__(self):

        # ROS initialization
        SingleAgentEnvironmentNode.__init__(
            self,
            environment_name='pose_sensor_and_actuator_test_environment',
            service_msg_type=PoseSensorAndActuatorTestEnvironmentStep,
        )


    def convert_action_to_request(self, action: np.ndarray) -> PoseSensorAndActuatorTestEnvironmentStep.Request:
        
        # action = np.array([x, y, z, roll, pitch, yaw]) in meters and radians
        ros_position = action[:3]
        ros_euler_angles = action[3:]

        # Convert the position from ROS to Unity coordinate system
        unity_position = PoseConverter.ros_to_unity_vector(ros_position)

        # Convert the orientation from euler angles to quaternion
        unity_orientation, _ = PoseConverter.ros_euler_to_unity_quaternion(ros_euler_angles)

        # Convert the action to ROS request format
        self._request.agent_action.target_pose.position.x = unity_position[0]
        self._request.agent_action.target_pose.position.y = unity_position[1]
        self._request.agent_action.target_pose.position.z = unity_position[2]

        self._request.agent_action.target_pose.orientation.x = unity_orientation[0]
        self._request.agent_action.target_pose.orientation.y = unity_orientation[1]
        self._request.agent_action.target_pose.orientation.z = unity_orientation[2]
        self._request.agent_action.target_pose.orientation.w = unity_orientation[3]

        return self._request
    

    def convert_response_to_state(self, response) -> np.ndarray:

        # Store the response
        self._response = response

        # Convert position from Unity to ROS coordinate system
        ros_position = PoseConverter.ros_to_unity_position(np.array([
            response.agent_state.pose.position.x,
            response.agent_state.pose.position.y,
            response.agent_state.pose.position.z,
        ]))

        # Convert orientation from Unity to ROS coordinate system
        unity_orientation = np.array([
            response.agent_state.pose.orientation.x,
            response.agent_state.pose.orientation.y,
            response.agent_state.pose.orientation.z,
            response.agent_state.pose.orientation.w,
        ])

        # Convert the orientation from quaternion to euler angles
        ros_euler_angles, _ = PoseConverter.unity_quaternion_to_ros_euler(unity_orientation)

        # Convert the response to numpy array
        state = np.array([
            ros_position[0],        # x
            ros_position[1],        # y
            ros_position[2],        # z
            ros_euler_angles[0],    # roll
            ros_euler_angles[1],    # pitch
            ros_euler_angles[2],    # yaw
        ])

        return state
    

    def render(self, state: np.ndarray):

        print(
            f'Position:\n'
            f'  x: {state[0]:.2f} m\n'
            f'  y: {state[1]:.2f} m\n'
            f'  z: {state[2]:.2f} m\n'
            f'Orientation:\n'
            f'  roll : {PoseConverter.radian_to_degree(state[3]):.2f} deg\n'
            f'  pitch: {PoseConverter.radian_to_degree(state[4]):.2f} deg\n'
            f'  yaw  : {PoseConverter.radian_to_degree(state[5]):.2f} deg'
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
        action = state
        env.render(state)

    env.close()


if __name__ == '__main__':
    main()
