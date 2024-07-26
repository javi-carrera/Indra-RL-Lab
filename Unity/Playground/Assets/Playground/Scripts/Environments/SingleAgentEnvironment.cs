using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

public abstract class SingleAgentEnvironment<TRequest, TResponse> : MonoBehaviour where TRequest : Message, new() where TResponse : Message, new() {

    [Header("ROS Connection")]
    public string environmentName;
    private string _serviceName;
    private ROSConnection _ROS;


    void Start() {

        // ROS connection
        _ROS = ROSConnection.GetOrCreateInstance();
        _serviceName = $"/{environmentName}";
        _ROS.ImplementService<TRequest, TResponse>(_serviceName, ServiceCallback);

    }

    protected abstract TResponse ServiceCallback(TRequest request);

}