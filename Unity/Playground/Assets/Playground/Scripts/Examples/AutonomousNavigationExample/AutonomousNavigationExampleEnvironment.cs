using System.Collections;
using System.Collections.Generic;
using UnityEngine;

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
    public AgentType agent;


    public override void Initialize(uint environmentId = 0) {
        
        // Append the agent to the list
        _agents = new List<IAgent> {
            agent
        };

        // Initialize the environment
        base.Initialize(environmentId);
        
    }


    protected override ActionResponse Action(ActionRequest request) {

        // Send the action to the agent
        agent.Action(request.action);

        ActionResponse response = new() {
            timestamp = GetCurrentTimestamp()
        };

        return response;
    }


    protected override StateResponse State(StateRequest request) {

        // Get the state from the agent
        StateResponse response = new() {
            state = agent.State(),
            timestamp = GetCurrentTimestamp()
        };

        return response;
    }


    protected override ResetResponse EnvironmentReset(ResetRequest request) {

        // Reset the agent
        ResetResponse response = new() {
            state = agent.ResetAgent(request.reset_action),
            timestamp = GetCurrentTimestamp()
        };

        return response;
    }
}
