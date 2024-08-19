using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using System.Collections;
using System.Collections.Generic;

using RosMessageTypes.BuiltinInterfaces;
using System.Threading.Tasks;


public interface ISingleAgentEnvironment {

    public string EnvironmentName { get; }
    public List<IAgent> Agents { get; }
    void Initialize(uint environmentId = 0);
}



public abstract class SingleAgentEnvironment<TStepRequest, TStepResponse, TResetRequest, TResetResponse> : MonoBehaviour, ISingleAgentEnvironment
    where TStepRequest : Message, new()
    where TStepResponse : Message, new()
    where TResetRequest : Message, new()
    where TResetResponse : Message, new() {


    [Header("ROS Connection")]
    public string environmentName;
    private ROSConnection _ROS;
    private uint _environmentId;
    private string _stepServiceName;
    private string _resetServiceName;
    private bool _isInitialized = false;

    [Header("Simulation")]
    public bool freeze;
    public float sampleTime = 0.0f;
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

        _stepServiceName = $"/{environmentName}_{environmentId}/step";
        // _stateServiceName = $"/{environmentName}_{environmentId}/state";
        _resetServiceName = $"/{environmentName}_{environmentId}/reset";

        _ROS.ImplementService<TStepRequest, TStepResponse>(_stepServiceName, StepServiceCallback);
        // _ROS.ImplementService<TStateRequest, TStateResponse>(_stateServiceName, StateServiceCallback);
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
    private async Task<TStepResponse> StepServiceCallback(TStepRequest request) {

        if (freeze) UnfreezeEnvironment();

        // Execute the action
        Action(request);
    
        // Wait for the sample time
        await Task.Delay((int)(sampleTime * 1000));

        // Get the state
        TStepResponse response = State();

        // Freeze the environment
        if (freeze) FreezeEnvironment();
        
        return response;
    }

    /// <summary>
    /// [TODO]
    /// </summary>
    private TResetResponse ResetServiceCallback(TResetRequest request) {

        TResetResponse response = ResetEnvironment(request);
        
        // Unfreeze the environment
        if (freeze) UnfreezeEnvironment();

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
    protected abstract void Action(TStepRequest request);

    /// <summary>
    /// [TODO]
    /// </summary>
    protected abstract TStepResponse State();

    /// <summary>
    /// [TODO]
    /// </summary>
    protected abstract TResetResponse ResetEnvironment(TResetRequest request);


    // Implement ISingleAgentEnvironment
    string ISingleAgentEnvironment.EnvironmentName => environmentName;
    List<IAgent> ISingleAgentEnvironment.Agents => _agents;

}