using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.InterfacesPkg;

public class AutonomousNavigationExampleEnvironment : SingleAgentEnvironment<AutonomousNavigationExampleEnvironmentStepRequest, AutonomousNavigationExampleEnvironmentStepResponse> {

    [Header("Autonomous Navigation Environment")]
    [SerializeField]
    private AutonomousNavigationExampleAgent _agent;
    
    override protected AutonomousNavigationExampleEnvironmentStepResponse ServiceCallback(AutonomousNavigationExampleEnvironmentStepRequest request) {

        // Agent logic here
        AutonomousNavigationExampleEnvironmentStepResponse response = new AutonomousNavigationExampleEnvironmentStepResponse();

        if (request.reset){
            // Reset the agent
            _agent.ResetAgent(request.agent_action);
        }

        else {
            // Send the action to the agent
            _agent.PerformAction(request.agent_action);
            // TODO: Wait 'sample_time'
        }

        response.agent_state = _agent.UpdateAgentState();

        return response;
    }
}
