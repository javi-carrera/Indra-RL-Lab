import json
import os


class InterfacesGenerator:

    def __init__(self, interfaces_path: str, environment_json_file_name: str):

        self.interfaces_path = interfaces_path
        self.environment_json_file_name = environment_json_file_name



    def generate_interfaces(self):

        # Create the directories
        os.makedirs(f'{self.interfaces_path}/msg', exist_ok=True)
        os.makedirs(f'{self.interfaces_path}/srv', exist_ok=True)

        # Load the environment json file
        with open(f'{self.interfaces_path}/{self.environment_json_file_name}', 'r') as f:
            environment_json = json.load(f)

        # Generate the interfaces
        self.environment_name = environment_json['environmentName']

        for agent in environment_json['agents']:
            self.generate_agent_interfaces(agent)

        self.generate_environment_interfaces(environment_json)


    def generate_agent_interfaces(self, agent: dict):

            agent_name = agent['agentName']

            sensors = agent['sensors']
            state_actuators = agent['stateActuators']
            reset_actuators = agent['resetActuators']

            # Create the AgentAction msg interface
            with open(f'{self.interfaces_path}/msg/{self.environment_name}{agent_name}Action.msg', 'w') as f:
                
                for actuator in state_actuators:
                    f.write(f"{actuator['messageType']} {actuator['actuatorName']}\n")

            # Create the AgentState msg interface
            with open(f'{self.interfaces_path}/msg/{self.environment_name}{agent_name}State.msg', 'w') as f:
                
                for sensor in sensors:
                    f.write(f"{sensor['messageType']} {sensor['sensorName']}\n")

            # Create the AgentReset msg interface
            with open(f'{self.interfaces_path}/msg/{self.environment_name}{agent_name}Reset.msg', 'w') as f:
                
                for actuator in reset_actuators:
                    f.write(f"{actuator['messageType']} {actuator['actuatorName']}\n")


    def generate_environment_interfaces(self, environment: dict):

        # Create the EnvironmentAction srv interface
        with open(f'{self.interfaces_path}/srv/{self.environment_name}EnvironmentStep.srv', 'w') as f:
            
            f.write('builtin_interfaces/Time timestamp\n')

            for agent in environment['agents']:
                agent_name = agent['agentName']
                f.write(f'{self.environment_name}{agent_name}Action {agent_name.lower()}_action\n')

            f.write('---\n')
            f.write('builtin_interfaces/Time timestamp')

            for agent in environment['agents']:
                agent_name = agent['agentName']
                f.write(f'{self.environment_name}{agent_name}State {agent_name.lower()}_state\n')


        # Create the EnvironmentReset srv interface
        with open(f'{self.interfaces_path}/srv/{self.environment_name}EnvironmentReset.srv', 'w') as f:
            
            f.write('builtin_interfaces/Time timestamp\n')

            for agent in environment['agents']:
                agent_name = agent['agentName']
                f.write(f'{self.environment_name}{agent_name}Reset {agent_name.lower()}_reset_action\n')

            f.write('---\n')
            f.write('builtin_interfaces/Time timestamp')

            
        
            


if __name__ == '__main__':

    interfaces_generator = InterfacesGenerator(
        interfaces_path = 'ros/src/playground_pkg/playground_pkg/interfaces',
        environment_json_file_name = 'AutonomousNavigationExample.json'
    )

    interfaces_generator.generate_interfaces()
