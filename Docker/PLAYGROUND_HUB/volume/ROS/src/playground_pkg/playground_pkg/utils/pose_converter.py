from typing import Tuple
import numpy as np
import numpy as np
from scipy.spatial.transform import Rotation


class PoseConverter:

    # @staticmethod
    # def unity_to_ros_position(unity_position: np.ndarray) -> np.ndarray:

    #     """Convert position from Unity coordinate system to ROS coordinate system."""

    #     ros_position = np.array([
    #         unity_position[0],  # Forward (X) remains the same
    #         unity_position[2],  # Right (Z in Unity) becomes Right (Y in ROS)
    #         unity_position[1],  # Up (Y in Unity) becomes Up (Z in ROS)
    #     ])

    #     return ros_position


    @staticmethod
    def ros_to_unity_vector(vector: np.ndarray) -> np.ndarray:

        """Convert position from ROS coordinate system to Unity coordinate system."""

        return np.array([
            vector[0],  # Forward (X) remains the same
            vector[2],  # Right (Y in ROS) becomes Right (Z in Unity)
            vector[1],  # Up (Z in ROS) becomes Up (Y in Unity)
        ])
    
    
    @staticmethod
    def ros_to_unity_euler_vector(vector: np.ndarray) -> np.ndarray:

        """Convert position from ROS coordinate system to Unity coordinate system."""

        return np.array([
            -vector[0],  # Roll (X) remains the same (but sign changes)
            -vector[2],  # Pitch (Y in ROS) becomes Yaw (Z in Unity) (but sign changes)
            -vector[1],  # Yaw (Z in ROS) becomes Pitch (Y in Unity) (but sign changes)
        ])
    

    @staticmethod
    def ros_to_unity_quaternion(quaternion: np.ndarray) -> np.ndarray:
            
        # Rearrange the quaternion to match the Unity coordinate system
        quaternion = np.array([
            quaternion[0],
            quaternion[2],
            quaternion[1],
            quaternion[3],
        ])

        return quaternion


    @staticmethod
    def ros_euler_to_unity_quaternion(euler_angles: np.ndarray) -> Tuple[np.ndarray, Rotation]:

        # Change the sign of the euler angles to match the Unity rotation direction
        euler_angles = -euler_angles

        # Get the rotation from euler angles
        rotation = Rotation.from_euler('xyz', euler_angles, degrees=False)

        # Get the quaternion from rotation
        quaternion = rotation.as_quat()

        # Rearrange the quaternion to match the Unity coordinate system
        quaternion = PoseConverter.ros_to_unity_quaternion(quaternion)

        return quaternion, rotation
    

    @staticmethod
    def unity_quaternion_to_ros_euler(quaternion: np.ndarray) -> Tuple[np.ndarray, Rotation]:

        # Rearrange the quaternion to match the ROS coordinate system
        quaternion = PoseConverter.ros_to_unity_quaternion(quaternion)
        
        # Get the rotation from quaternion
        rotation = Rotation.from_quat(quaternion)

        # Get the euler angles from rotation
        euler_angles = rotation.as_euler('xyz', degrees=False)

        # Change the sign of the euler angles to match the ROS rotation direction
        euler_angles = -euler_angles

        return euler_angles, rotation


    @staticmethod
    def radian_to_degree(radian: np.ndarray) -> np.ndarray:
        return np.degrees(radian)
    
    @staticmethod
    def degree_to_radian(degree: np.ndarray) -> np.ndarray:
        return np.radians(degree)