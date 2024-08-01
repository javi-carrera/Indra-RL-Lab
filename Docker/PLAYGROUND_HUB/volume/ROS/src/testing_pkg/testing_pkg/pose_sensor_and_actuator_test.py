from typing import Tuple, Type

import gymnasium as gym
import numpy as np
import rclpy

from playground_pkg.single_agent_environment_node import SingleAgentEnvironmentNode
from playground_pkg.utils.pose_converter import PoseConverter
from interfaces_pkg.srv import PoseSensorAndActuatorTestEnvironmentAction, PoseSensorAndActuatorTestEnvironmentState, PoseSensorAndActuatorTestEnvironmentReset

import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R


class PoseSensorAndActuatorTestEnvironment(SingleAgentEnvironmentNode):

    def __init__(self):

        # ROS initialization
        SingleAgentEnvironmentNode.__init__(
            self,
            environment_name='pose_sensor_and_actuator_test_environment',
            action_service_msg_type=PoseSensorAndActuatorTestEnvironmentAction,
            state_service_msg_type=PoseSensorAndActuatorTestEnvironmentState,
            reset_service_msg_type=PoseSensorAndActuatorTestEnvironmentReset,
            sample_time=0.0
        )


    def convert_action_to_request(self, action: np.ndarray) -> PoseSensorAndActuatorTestEnvironmentAction.Request:
        
        # action = np.array([x, y, z, roll, pitch, yaw]) in meters and radians
        ros_position = action[:3]
        ros_euler_angles = action[3:]

        # Convert the position from ROS to Unity coordinate system
        unity_position = PoseConverter.ros_to_unity_vector(ros_position)

        # Convert the orientation from euler angles to quaternion
        unity_orientation, _ = PoseConverter.ros_euler_to_unity_quaternion(ros_euler_angles)

        # Convert the action to ROS request format
        self.action_request.action.target_pose.position.x = unity_position[0]
        self.action_request.action.target_pose.position.y = unity_position[1]
        self.action_request.action.target_pose.position.z = unity_position[2]

        self.action_request.action.target_pose.orientation.x = unity_orientation[0]
        self.action_request.action.target_pose.orientation.y = unity_orientation[1]
        self.action_request.action.target_pose.orientation.z = unity_orientation[2]
        self.action_request.action.target_pose.orientation.w = unity_orientation[3]

        return self.action_request
    

    def convert_response_to_state(self, response) -> dict:

        # Convert position from Unity to ROS coordinate system
        ros_position = PoseConverter.ros_to_unity_vector(np.array([
            response.state.pose.position.x,
            response.state.pose.position.y,
            response.state.pose.position.z,
        ]))

        # Convert orientation from Unity to ROS coordinate system
        unity_orientation = np.array([
            response.state.pose.orientation.x,
            response.state.pose.orientation.y,
            response.state.pose.orientation.z,
            response.state.pose.orientation.w,
        ])

        # Convert the orientation from quaternion to euler angles
        ros_euler_angles, _ = PoseConverter.unity_quaternion_to_ros_euler(unity_orientation)

        # Convert the response to dict
        state = {
            'x': ros_position[0],
            'y': ros_position[1],
            'z': ros_position[2],
            'roll': ros_euler_angles[0],
            'pitch': ros_euler_angles[1],
            'yaw': ros_euler_angles[2],
        }

        return state
    

    def convert_reset_to_request(self) -> Type:
        return self.reset_request
    

    def render(self, state: np.ndarray):

        print(
            f'Position:\n'
            f'  x: {state["x"]:.2f} m\n'
            f'  y: {state["y"]:.2f} m\n'
            f'  z: {state["z"]:.2f} m\n'
            f'Orientation:\n'
            f'  roll : {PoseConverter.radian_to_degree(state["roll"]):.2f} deg\n'
            f'  pitch: {PoseConverter.radian_to_degree(state["pitch"]):.2f} deg\n'
            f'  yaw  : {PoseConverter.radian_to_degree(state["yaw"]):.2f} deg'
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
        action = np.array([state['x'], state['y'], state['z'], state['roll'], state['pitch'], state['yaw']])
        env.render(state)

    env.close()


if __name__ == '__main__':
    main()
