using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using System.Collections;
using System.Collections.Generic;

using RosMessageTypes.BuiltinInterfaces;




public abstract class SingleAgentEnvironment<TActionRequest, TActionResponse, TStateRequest, TStateResponse, TResetRequest, TResetResponse> : MonoBehaviour
    where TActionRequest : Message, new()
    where TActionResponse : Message, new()
    where TStateRequest : Message, new()
    where TStateResponse : Message, new()
    where TResetRequest : Message, new()
    where TResetResponse : Message, new() {


    [Header("ROS Connection")]
    public string environmentName;
    private string _actionServiceName;
    private string _stateServiceName;
    private string _resetServiceName;
    private ROSConnection _ROS;

    [Header("Simulation")]
    public bool freeze = false;
    public float timeScale;


    protected void Start() {

        // ROS connection
        _ROS = ROSConnection.GetOrCreateInstance();

        _actionServiceName = $"/{environmentName}/action";
        _stateServiceName = $"/{environmentName}/state";
        _resetServiceName = $"/{environmentName}/reset";

        _ROS.ImplementService<TActionRequest, TActionResponse>(_actionServiceName, ActionServiceCallback);
        _ROS.ImplementService<TStateRequest, TStateResponse>(_stateServiceName, StateServiceCallback);
        _ROS.ImplementService<TResetRequest, TResetResponse>(_resetServiceName, ResetServiceCallback);

        // Simulation settings initialization
        timeScale = Time.timeScale;

    }

    /// <summary>
    /// [TODO]
    /// </summary>
    private TActionResponse ActionServiceCallback(TActionRequest request) {

        // Unfreeze the environment
        if (freeze) {
            Time.timeScale = timeScale;
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
            Time.timeScale = 0.0f;
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
            Time.timeScale = timeScale;
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

    /// <summary>
    /// [TODO]
    /// </summary>
    abstract protected TActionResponse Action(TActionRequest request);

    /// <summary>
    /// [TODO]
    /// </summary>
    abstract protected TStateResponse State(TStateRequest request);

    /// <summary>
    /// [TODO]
    /// </summary>
    abstract protected TResetResponse EnvironmentReset(TResetRequest request);

}