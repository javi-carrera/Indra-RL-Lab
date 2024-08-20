using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using AgentType = AutonomousNavigationExampleAgent;
using StateRequest = RosMessageTypes.InterfacesPkg.AutonomousNavigationExampleEnvironmentStepRequest;
using StateResponse = RosMessageTypes.InterfacesPkg.AutonomousNavigationExampleEnvironmentStepResponse;
using ResetRequest = RosMessageTypes.InterfacesPkg.AutonomousNavigationExampleEnvironmentResetRequest;
using ResetResponse = RosMessageTypes.InterfacesPkg.AutonomousNavigationExampleEnvironmentResetResponse;


public class AutonomousNavigationExampleEnvironment : Environment<
    StateRequest,
    StateResponse,
    ResetRequest,
    ResetResponse> {
    
    [Header("Agent")]
    public AgentType agent;


    public override void Initialize() {
        
        // Append the agent to the list
        _agents = new List<IAgent> {
            agent
        };

        // Initialize the environment
        base.Initialize();
        
    }


    protected override void Action(StateRequest request) {

        // Send the action to the agent
        agent.Action(request.action);
        
    }


    protected override StateResponse State() {

        // Get the state from the agent
        StateResponse response = new() {
            state = agent.State(),
            timestamp = GetCurrentTimestamp()
        };

        return response;
    }


    protected override ResetResponse ResetEnvironment(ResetRequest request) {

        // Reset the agent
        ResetResponse response = new() {
            state = agent.ResetAgent(request.reset_action),
            timestamp = GetCurrentTimestamp()
        };

        return response;
    }
}
