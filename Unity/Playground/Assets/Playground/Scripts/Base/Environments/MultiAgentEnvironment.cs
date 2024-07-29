using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.InterfacesPkg;

public class MultiAgentEnvironment : MonoBehaviour {
    
    ROSConnection _ROS;
    public string serviceName;

    void Start() {

        _ROS = ROSConnection.GetOrCreateInstance();
        _ROS.ImplementService<EnvironmentStepRequest, EnvironmentStepResponse>(serviceName, ServiceCallback);
    }

    private EnvironmentStepResponse ServiceCallback(EnvironmentStepRequest request) {

        EnvironmentStepResponse response = new EnvironmentStepResponse();

        string msg = $"Incoming request with action: {request.agent_action}\n";
        Debug.Log(msg);

        return response;
    }
}