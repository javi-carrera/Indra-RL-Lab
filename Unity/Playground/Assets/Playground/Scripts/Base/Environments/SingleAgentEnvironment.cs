using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using System.Collections;
using System.Collections.Generic;

using RosMessageTypes.BuiltinInterfaces;


public interface ISingleAgentEnvironment {
    void Initialize(uint environmentId);
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


    protected void Start() {

        if (!_isInitialized) {
            Initialize();
        }
    }


    public void Initialize(uint environmentId = 0) {

        // ROS connection initialization
        _ROS = ROSConnection.GetOrCreateInstance();
        _environmentId = environmentId;

        environmentName = $"{environmentName}_{_environmentId}";

        _actionServiceName = $"/{environmentName}/action";
        _stateServiceName = $"/{environmentName}/state";
        _resetServiceName = $"/{environmentName}/reset";

        _ROS.ImplementService<TActionRequest, TActionResponse>(_actionServiceName, ActionServiceCallback);
        _ROS.ImplementService<TStateRequest, TStateResponse>(_stateServiceName, StateServiceCallback);
        _ROS.ImplementService<TResetRequest, TResetResponse>(_resetServiceName, ResetServiceCallback);

        // Simulation initialization
        Time.timeScale = timeScale;

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

    private void FreezeEnvironment() {
        Time.timeScale = 0.0f;
    }

    private void UnfreezeEnvironment() {
        Time.timeScale = timeScale;
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

}