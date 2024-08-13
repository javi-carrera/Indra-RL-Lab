using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using System.Collections;
using System.Collections.Generic;

using RosMessageTypes.BuiltinInterfaces;


public interface ISingleAgentEnvironment {

    public string EnvironmentName { get; }
    public List<IAgent> Agents { get; }
    void Initialize(uint environmentId = 0);
}



public abstract class SingleAgentEnvironment<TActionRequest, TActionResponse, TStateRequest, TStateResponse, TResetRequest, TResetResponse> : MonoBehaviour, ISingleAgentEnvironment
    where TActionRequest : Message, new()
    where TActionResponse : Message, new()
    where TStateRequest : Message, new()
    where TStateResponse : Message, new()
    where TResetRequest : Message, new()
    where TResetResponse : Message, new() {


    [Header("ROS Connection")]
    public string environmentName;
    private ROSConnection _ROS;
    private uint _environmentId;
    private string _actionServiceName;
    private string _stateServiceName;
    private string _resetServiceName;
    private bool _isInitialized = false;

    [Header("Simulation")]
    public bool freeze;
    public float timeScale = 1.0f;

    [Header("Agent")]
    protected List<IAgent> _agents;


    protected void Start() {

        if (!_isInitialized) {
            Initialize();
        }
    }


    public virtual void Initialize(uint environmentId = 0) {

        // ROS connection initialization
        _ROS = ROSConnection.GetOrCreateInstance();
        _environmentId = environmentId;

        _actionServiceName = $"/{environmentName}_{environmentId}/action";
        _stateServiceName = $"/{environmentName}_{environmentId}/state";
        _resetServiceName = $"/{environmentName}_{environmentId}/reset";

        _ROS.ImplementService<TActionRequest, TActionResponse>(_actionServiceName, ActionServiceCallback);
        _ROS.ImplementService<TStateRequest, TStateResponse>(_stateServiceName, StateServiceCallback);
        _ROS.ImplementService<TResetRequest, TResetResponse>(_resetServiceName, ResetServiceCallback);

        // Simulation initialization
        Time.timeScale = timeScale;

        // Initialize agents list
        foreach (IAgent agent in _agents) {
            agent.Initialize();
        }

        _isInitialized = true;

    }

    /// <summary>
    /// [TODO]
    /// </summary>
    private TActionResponse ActionServiceCallback(TActionRequest request) {

        // Unfreeze the environment
        if (freeze) {
            UnfreezeEnvironment();
        }
        
        return Action(request);
    }

    /// <summary>
    /// [TODO]
    /// </summary>
    private TStateResponse StateServiceCallback(TStateRequest request) {

        TStateResponse response = State(request); 

        // Freeze the environment
        if (freeze) {
            FreezeEnvironment();
        }

        return response;
    }

    /// <summary>
    /// [TODO]
    /// </summary>
    private TResetResponse ResetServiceCallback(TResetRequest request) {

        TResetResponse response = EnvironmentReset(request);
        
        // Unfreeze the environment
        if (freeze){
            UnfreezeEnvironment();
        }

        return response;
    }


    private void FreezeEnvironment() {
        Time.timeScale = 0.0f;
    }

    private void UnfreezeEnvironment() {
        Time.timeScale = timeScale;
    }

    /// <summary>
    /// [TODO]
    /// </summary>
    protected TimeMsg GetCurrentTimestamp() {

        int seconds;
        uint nanoseconds;
        double totalSeconds;

        // Get the current time
        DateTime currentTime = DateTime.UtcNow;

        totalSeconds = currentTime.Subtract(new DateTime(1970, 1, 1)).TotalSeconds;
        seconds = (int)totalSeconds;
        nanoseconds = (uint)((totalSeconds - seconds) * 1e9);

        // Get the current timestamp
        return new TimeMsg {
            sec = seconds,
            nanosec = nanoseconds
        };
    }

    /// <summary>
    /// [TODO]
    /// </summary>
    protected abstract TActionResponse Action(TActionRequest request);

    /// <summary>
    /// [TODO]
    /// </summary>
    protected abstract TStateResponse State(TStateRequest request);

    /// <summary>
    /// [TODO]
    /// </summary>
    protected abstract TResetResponse EnvironmentReset(TResetRequest request);


    // Implement ISingleAgentEnvironment
    string ISingleAgentEnvironment.EnvironmentName => environmentName;
    List<IAgent> ISingleAgentEnvironment.Agents => _agents;

}