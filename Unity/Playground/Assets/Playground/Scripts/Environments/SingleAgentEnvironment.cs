using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.InterfacesPkg;
using System;
using Unity.VisualScripting;

public class SingleAgentEnvironment : MonoBehaviour {

    [Header("ROS Connection")]
    public string environmentName;
    private string _serviceName;
    private ROSConnection _ROS;

    // Environment specific variables
    private Agent _agent;

    void Start() {

        // ROS connection
        _ROS = ROSConnection.GetOrCreateInstance();
        _serviceName = $"/{environmentName}";
        _ROS.ImplementService<EnvironmentStepRequest, EnvironmentStepResponse>(_serviceName, ServiceCallback);

        // Environment specific initializations
        _agent = GetComponentInChildren<Agent>();

    }

    private EnvironmentStepResponse ServiceCallback(EnvironmentStepRequest request) {

        // Agent logic here
        EnvironmentStepResponse response = new EnvironmentStepResponse();

        if (request.reset){
            // Reset the agent
            _agent.ResetAgent();
        }

        else{
            // Send the action to the agent
            _agent.SetActuatorDataFromAction(request.agent_action);
            // TODO: Wait 'sample_time'
        }

        response.agent_state = _agent.GetStateFromSensorData();

        return response;
    }
}