using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.InterfacesPkg;

public class LidarSensorTestEnvironment : SingleAgentEnvironment<LidarSensorTestEnvironmentStepRequest, LidarSensorTestEnvironmentStepResponse> {

    [Header("LiDAR Sensor Test Environment")]
    public LidarSensorTestAgent _agent;
    
    override protected LidarSensorTestEnvironmentStepResponse ServiceCallback(LidarSensorTestEnvironmentStepRequest request) {

        // Agent logic here
        LidarSensorTestEnvironmentStepResponse response = new LidarSensorTestEnvironmentStepResponse();

        if (request.reset) {
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
