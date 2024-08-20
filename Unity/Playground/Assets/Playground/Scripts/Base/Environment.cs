using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using System.Collections;
using System.Collections.Generic;

using RosMessageTypes.BuiltinInterfaces;
using System.Threading.Tasks;


public interface IEnvironment {

    public string EnvironmentName { get; }
    public List<IAgent> Agents { get; }
    void Initialize();
}



public abstract class Environment<TStepRequest, TStepResponse, TResetRequest, TResetResponse> : MonoBehaviour, IEnvironment
    where TStepRequest : Message, new()
    where TStepResponse : Message, new()
    where TResetRequest : Message, new()
    where TResetResponse : Message, new() {


    [Header("ROS Connection")]
    public string environmentName;
    private ROSConnection _ROS;
    public string rosIPAddress;
    public int rosPort;
    private uint _environmentId;
    private string _stepServiceName;
    private string _resetServiceName;
    private bool _isInitialized = false;

    [Header("Simulation")]
    public bool pause;
    public float sampleTime;
    public float timeScale;


    [Header("Agent")]
    protected List<IAgent> _agents;


    protected void Start() {

        string[] args = System.Environment.GetCommandLineArgs();

        for (int i = 0; i < args.Length; i++) {

            if (args[i] == "--ros-ip") {
                rosIPAddress = args[i + 1];
            }
            
            else if (args[i] == "--ros-port") {
                rosPort = int.Parse(args[i + 1]);
            }

            else if (args[i] == "--environment-id") {
                _environmentId = uint.Parse(args[i + 1]);
            }
        }

        if (!_isInitialized) {
            Initialize();
        }
    }


    public virtual void Initialize() {

        // ROS connection initialization
        _ROS = ROSConnection.GetOrCreateInstance();
        _ROS.RosIPAddress = rosIPAddress;
        _ROS.RosPort = rosPort;

        _stepServiceName = $"/{environmentName}_{_environmentId}/step";
        _resetServiceName = $"/{environmentName}_{_environmentId}/reset";

        _ROS.ImplementService<TStepRequest, TStepResponse>(_stepServiceName, StepServiceCallback);
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

        // Resume the environment
        if (pause) Resume();

        // Execute the action
        Action(request);
    
        // Wait for the sample time
        await Task.Delay((int)(sampleTime * 1000));

        // Get the state
        TStepResponse response = State();

        // Pause the environment
        if (pause) Pause();
        
        return response;
        
    }

    /// <summary>
    /// [TODO]
    /// </summary>
    private TResetResponse ResetServiceCallback(TResetRequest request) {

        // Resume the environment
        if (pause) Resume();


        // Reset the environment
        TResetResponse response = ResetEnvironment(request);
        
        // Pause the environment
        if (pause) Pause();


        return response;
    }


    private void Pause() => Time.timeScale = 0.0f;

    private void Resume() => Time.timeScale = timeScale;

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
    string IEnvironment.EnvironmentName => environmentName;
    List<IAgent> IEnvironment.Agents => _agents;
    void IEnvironment.Initialize() => Initialize();

}