using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using AgentType = ShootingExampleAgent;
using StateRequest = RosMessageTypes.InterfacesPkg.ShootingExampleEnvironmentStepRequest;
using StateResponse = RosMessageTypes.InterfacesPkg.ShootingExampleEnvironmentStepResponse;
using ResetRequest = RosMessageTypes.InterfacesPkg.ShootingExampleEnvironmentResetRequest;
using ResetResponse = RosMessageTypes.InterfacesPkg.ShootingExampleEnvironmentResetResponse;
using RosMessageTypes.BuiltinInterfaces;


public class ShootingExampleEnvironment : Environment<
    StateRequest,
    StateResponse,
    ResetRequest,
    ResetResponse> {
    
    [Header("Agent")]
    public AgentType agent;


    protected override void InitializeEnvironment() {

        // Append the agent to the list
        _agents = new List<IAgent> {
            agent
        };

        // Initialize agents list
        foreach (IAgent agent in _agents) {
            agent.Initialize();
        }
        
    }


    protected override void Action(StateRequest request) {

        // Send the action to the agent
        agent.Action(request.action);
        
    }


    protected override StateResponse State(TimeMsg requestReceivedTimestamp) {

        // Get the state from the agent
        StateResponse response = new() {
            state = agent.State(),
            request_received_timestamp = requestReceivedTimestamp,
            response_sent_timestamp = GetCurrentTimestamp()
        };

        return response;
    }


    protected override ResetResponse ResetEnvironment(ResetRequest request, TimeMsg requestReceivedTimestamp) {

        // Reset the agent
        ResetResponse response = new() {
            state = agent.ResetAgent(request.reset_action),
            request_received_timestamp = requestReceivedTimestamp,
            response_sent_timestamp = GetCurrentTimestamp()
        };

        return response;
    }
}
