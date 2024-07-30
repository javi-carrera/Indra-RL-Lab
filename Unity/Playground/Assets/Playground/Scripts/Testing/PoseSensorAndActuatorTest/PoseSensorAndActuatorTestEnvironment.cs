using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.InterfacesPkg;

public class PoseSensorAndActuatorTestEnvironment : SingleAgentEnvironment<PoseSensorAndActuatorTestEnvironmentStepRequest, PoseSensorAndActuatorTestEnvironmentStepResponse> {

    [Header("Pose Sensor And Actuator Test Environment")]
    public PoseSensorAndActuatorTestAgent _agent;
    
    override protected PoseSensorAndActuatorTestEnvironmentStepResponse ServiceCallback(PoseSensorAndActuatorTestEnvironmentStepRequest request) {

        // Agent logic here
        PoseSensorAndActuatorTestEnvironmentStepResponse response = new PoseSensorAndActuatorTestEnvironmentStepResponse();

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
