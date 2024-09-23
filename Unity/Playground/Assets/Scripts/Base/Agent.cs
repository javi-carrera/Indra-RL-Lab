using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.InterfacesPkg;
using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;



public interface IAgent {
    
    public List<ISensor> Sensors { get; }
    public List<IActuator> StateActuators { get; }

    void Initialize();
    void Action(Message action);
    Message State();
    Message ResetAgent();
    
}


public abstract class Agent<TActionMsg, TStateMsg> : MonoBehaviour, IAgent
    where TActionMsg : Message, new()
    where TStateMsg : Message, new() {

    
    protected List<ISensor> _sensors;
    protected List<IActuator> _actuators;

    [Header("Debug")]
    public bool overrideAction;
    public bool overrideReset;


    private void Update() {

        if (Input.GetKeyDown(KeyCode.O)) {
            overrideAction = !overrideAction;
            overrideReset = !overrideReset;
        }
        
        if (overrideAction) OverrideAction();
    }


    /// <summary>
    /// [TODO]
    /// </summary>
    public abstract void Initialize();

    /// <summary>
    /// [TODO]
    /// </summary>
    public abstract void OverrideAction();

    /// <summary>
    /// [TODO]
    /// </summary>
    public abstract void Action(TActionMsg action);

    /// <summary>
    /// [TODO]
    /// </summary>
    public abstract TStateMsg State();

    /// <summary>
    /// [TODO]
    /// </summary>
    public abstract TStateMsg ResetAgent();


    // Implement IAgent
    List<ISensor> IAgent.Sensors => _sensors;
    List<IActuator> IAgent.StateActuators => _actuators;

    void IAgent.Initialize() => Initialize();
    void IAgent.Action(Message action) => Action((TActionMsg)action);
    Message IAgent.State() => State();
    Message IAgent.ResetAgent() => ResetAgent();



}