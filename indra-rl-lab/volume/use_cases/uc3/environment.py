# Project: Indra-RL-Lab
# File: environment.py
# Authors: Javier Carrera
# License: Apache 2.0 (refer to LICENSE file in the project root)


from interfaces_pkg.msg import UC3AgentState
from interfaces_pkg.srv import UC3EnvironmentStep, UC3EnvironmentReset
from rl_pkg import EnvironmentNode


class UC3Environment(EnvironmentNode):

    def __init__(self, environment_id: int):

        # EnvironmentNode (ROS)
        EnvironmentNode.__init__(
            self,
            environment_name="uc3_environment",
            environment_id=environment_id,
            step_service_msg_type=UC3EnvironmentStep,
            reset_service_msg_type=UC3EnvironmentReset,
        )
        
        # Gymasium
        self.action_space = None
        self.observation_space = None
        self.reward_range = None

        # Environment parameters
        self.MIN_LINEAR_VELOCITY = -5.0
        self.MAX_LINEAR_VELOCITY = 5.0
        self.MAX_YAW_RATE = 5.0
        self.MAX_TURRET_ROTATION_SPEED = 5.5
        self.MAX_EPISODE_STEPS = 1024

        self.current_target_distance = None

    @property
    def reset_request(self) -> UC3EnvironmentReset.Request:
        return self._reset_request
    
    @property
    def step_request(self) -> UC3EnvironmentStep.Request:
        return self._step_request
    
    @property
    def step_response(self) -> UC3EnvironmentStep.Response:
        return self._step_response

    def reset_environment_variables(self):
        self.current_target_distance = None

    def terminated(self, state: UC3AgentState) -> bool:
        has_died = state.tank.health_info.health <= 0.0
        has_target_died = state.target_tank.health_info.health <= 0.0

        return has_died or has_target_died

    def truncated(self, state: UC3AgentState) -> bool:
        return self.n_step > self.MAX_EPISODE_STEPS

    def info(self, state: UC3AgentState) -> dict:
        return {}
    

