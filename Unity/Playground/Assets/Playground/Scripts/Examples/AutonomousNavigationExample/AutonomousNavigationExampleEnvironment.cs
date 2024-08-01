using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.InterfacesPkg;


using AgentType = AutonomousNavigationExampleAgent;
using ActionRequest = RosMessageTypes.InterfacesPkg.AutonomousNavigationExampleEnvironmentActionRequest;
using ActionResponse = RosMessageTypes.InterfacesPkg.AutonomousNavigationExampleEnvironmentActionResponse;
using StateRequest = RosMessageTypes.InterfacesPkg.AutonomousNavigationExampleEnvironmentStateRequest;
using StateResponse = RosMessageTypes.InterfacesPkg.AutonomousNavigationExampleEnvironmentStateResponse;
using ResetRequest = RosMessageTypes.InterfacesPkg.AutonomousNavigationExampleEnvironmentResetRequest;
using ResetResponse = RosMessageTypes.InterfacesPkg.AutonomousNavigationExampleEnvironmentResetResponse;


public class AutonomousNavigationExampleEnvironment : SingleAgentEnvironment<
    ActionRequest,
    ActionResponse,
    StateRequest,
    StateResponse,
    ResetRequest,
    ResetResponse> {
    

    [Header("Agent")]
    [SerializeField]
    private AgentType _agent;


    protected override ActionResponse Action(ActionRequest request) {

        // Send the action to the agent
        _agent.Action(request.action);

        ActionResponse response = new ActionResponse{
            timestamp = GetCurrentTimestamp()
        };

        return response;
    }

    protected override StateResponse State(StateRequest request) {

        // Get the state from the agent
        StateResponse response = new StateResponse {
            state = _agent.State(),
            timestamp = GetCurrentTimestamp()
        };

        return response;
    }

    protected override ResetResponse EnvironmentReset(ResetRequest request) {

        // Reset the agent
        ResetResponse response = new ResetResponse {
            
            state = _agent.ResetAgent(request.reset_action),
            timestamp = GetCurrentTimestamp()
        };

        return response;
    }
}
