using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using System.Collections;
using System.Collections.Generic;

using RosMessageTypes.BuiltinInterfaces;
using System.Threading.Tasks;
using System.Globalization;
using UnityEngine.UIElements;


public interface IEnvironment {

    public string RootServiceName { get; }
    public bool Pause { get; set; }
    public float SampleTime { get; set; }
    public float TimeScale { get; set; }
    public int RosPort { get; set; }

    
    public List<IAgent> Agents { get; }
    void Initialize();
}



public abstract class Environment<TStepRequest, TStepResponse, TResetRequest, TResetResponse> : MonoBehaviour, IEnvironment
    where TStepRequest : Message, new()
    where TStepResponse : Message, new()
    where TResetRequest : Message, new()
    where TResetResponse : Message, new() {


    [Header("ROS Connection")]
    private ROSConnection _ROS;
    public string rootServiceName;
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
    public float fixedDeltaTime;
    private bool _updateCalledBeforeStep = false;
    private bool _fixedUpdateCalledBeforeStep = false;


    [Header("Agent")]
    protected List<IAgent> _agents;


    protected void Start() {

        GetCommandLineArguments();

        if (!_isInitialized) Initialize();
    }

    protected void Update() {
        _updateCalledBeforeStep = true;
    }

    protected void FixedUpdate() {
        _fixedUpdateCalledBeforeStep = true;
    }

    protected void OnDestroy() {

        // Disconnect the ROS connection
        _ROS.Disconnect();

    }




    private void GetCommandLineArguments() {

        string[] args = System.Environment.GetCommandLineArgs();

        CultureInfo culture = CultureInfo.InvariantCulture;

        for (int i = 0; i < args.Length; i++) {

            switch (args[i]) {

                case "--ros-ip":
                    rosIPAddress = args[i + 1];
                    break;

                case "--ros-port":
                    rosPort = int.Parse(args[i + 1]);
                    break;

                case "--environment-id":
                    _environmentId = uint.Parse(args[i + 1]);
                    break;

                case "--pause":
                    pause = bool.Parse(args[i + 1]);
                    break;

                case "--sample-time":
                    sampleTime = float.Parse(args[i + 1], culture);
                    break;

                case "--time-scale":
                    timeScale = float.Parse(args[i + 1], culture);
                    break;
                
                default:
                    break;
            }
        }

    }


    public void Initialize() {

        if (_isInitialized) return;

        InitialzeROS();
        InitializeSimulation();
        InitializeEnvironment();

        _isInitialized = true;

    }

    private void InitialzeROS() {

        // Initialize ROS connection and assign the IP address and port
        _ROS = ROSConnection.GetOrCreateInstance();
        
        _ROS.RosIPAddress = rosIPAddress;
        _ROS.RosPort = rosPort;
        _ROS.Connect();

        // Define the service names
        _stepServiceName = $"/{rootServiceName}_{_environmentId}/step";
        _resetServiceName = $"/{rootServiceName}_{_environmentId}/reset";

        // Implement the services
        _ROS.ImplementService<TStepRequest, TStepResponse>(_stepServiceName, StepServiceCallback);
        _ROS.ImplementService<TResetRequest, TResetResponse>(_resetServiceName, ResetServiceCallback);

    }

    private void InitializeSimulation() {

        // Set the time scale
        Time.timeScale = timeScale;

        // Set the fixed delta time
        Time.fixedDeltaTime = fixedDeltaTime;
    }

    

    /// <summary>
    /// [TODO]
    /// </summary>
    private async Task<TStepResponse> StepServiceCallback(TStepRequest request) {

        TimeMsg requestReceivedTimestamp = GetCurrentTimestamp();

        // Resume the environment
        // if (pause) Resume();
        Resume();

        // Execute the action
        Action(request);

        // Wait for the update to be called
        int alreadyWaitedMilliseconds = 0;

        _updateCalledBeforeStep = false;
        _fixedUpdateCalledBeforeStep = false;

        while (!(_updateCalledBeforeStep && _fixedUpdateCalledBeforeStep)) {
        // while (!_updateCalledBeforeStep) {
            await Task.Delay(1);
            alreadyWaitedMilliseconds++;
        }
        
        int millisecondsToWait = (int)(sampleTime * 1000) - alreadyWaitedMilliseconds;
        
        // Wait for the sample time
        if (millisecondsToWait > 0)
            await Task.Delay(Mathf.CeilToInt(millisecondsToWait / timeScale));
        else if (pause)
            Debug.LogWarning($"The sample time {sampleTime} [s] is too short. Try increasing the sample time to {alreadyWaitedMilliseconds / 1000.0f} s or more.");
    

        // Get the state
        TStepResponse response = State(requestReceivedTimestamp);


        // Pause the environment
        if (pause) Pause();
        
        return response;
        
    }

    /// <summary>
    /// [TODO]
    /// </summary>
    private TResetResponse ResetServiceCallback(TResetRequest request) {

        TimeMsg requestReceivedTimestamp = GetCurrentTimestamp();

        // Resume the environment
        // if (pause) Resume();
        Resume();

        // Reset the environment
        TResetResponse response = ResetEnvironment(request, requestReceivedTimestamp);
        
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
    protected abstract void InitializeEnvironment();

    /// <summary>
    /// [TODO]
    /// </summary>
    protected abstract void Action(TStepRequest request);

    /// <summary>
    /// [TODO]
    /// </summary>
    protected abstract TStepResponse State(TimeMsg requestReceivedTimestamp);

    /// <summary>
    /// [TODO]
    /// </summary>
    protected abstract TResetResponse ResetEnvironment(TResetRequest request, TimeMsg requestReceivedTimestamp);


    // Implement ISingleAgentEnvironment
    string IEnvironment.RootServiceName => rootServiceName;
    bool IEnvironment.Pause { get => pause; set => pause = value; }
    float IEnvironment.SampleTime { get => sampleTime; set => sampleTime = value; }
    float IEnvironment.TimeScale { get => timeScale; set => timeScale = value; }
    int IEnvironment.RosPort { get => _ROS.RosPort; set => _ROS.RosPort = value; }
    List<IAgent> IEnvironment.Agents => _agents;
    
    void IEnvironment.Initialize() => Initialize();

}