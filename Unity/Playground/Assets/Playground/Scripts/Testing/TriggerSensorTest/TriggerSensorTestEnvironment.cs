using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.InterfacesPkg;

public class TriggerSensorTestEnvironment : SingleAgentEnvironment<TriggerSensorTestEnvironmentStepRequest, TriggerSensorTestEnvironmentStepResponse> {

    [Header("Trigger Sensor Test Environment")]
    public TriggerSensorTestAgent _agent;
    
    override protected TriggerSensorTestEnvironmentStepResponse ServiceCallback(TriggerSensorTestEnvironmentStepRequest request) {

        // Agent logic here
        TriggerSensorTestEnvironmentStepResponse response = new TriggerSensorTestEnvironmentStepResponse();

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
