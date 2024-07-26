using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.InterfacesPkg;

public class LidarTestEnvironment : SingleAgentEnvironment<LidarTestEnvironmentStepRequest, LidarTestEnvironmentStepResponse> {

    [Header("LiDAR Test Environment")]
    public LidarTestAgent _agent;
    
    override protected LidarTestEnvironmentStepResponse ServiceCallback(LidarTestEnvironmentStepRequest request) {

        // Agent logic here
        LidarTestEnvironmentStepResponse response = new LidarTestEnvironmentStepResponse();

        if (request.reset){
            // Reset the agent
            _agent.ResetAgent();
        }

        else{
            // Send the action to the agent
            _agent.PerformAction(request.agent_action);
            // TODO: Wait 'sample_time'
        }

        response.agent_state = _agent.UpdateAgentState();

        return response;
    }
}
