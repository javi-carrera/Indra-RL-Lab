using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.InterfacesPkg;
using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;



public interface IAgent {
    
    public List<ISensor> Sensors { get; }
    public List<IActuator> StateActuators { get; }
    public List<IActuator> ResetActuators { get; }

    void Initialize();
    void Action(Message action);
    Message State();
    Message ResetAgent(Message resetAction);
    
}


public abstract class Agent<TActionMsg, TStateMsg, TResetMsg> : MonoBehaviour, IAgent
    where TActionMsg : Message, new()
    where TStateMsg : Message, new()
    where TResetMsg : Message, new() {

    
    protected List<ISensor> _sensors;
    protected List<IActuator> _stateActuators;
    protected List<IActuator> _resetActuators;

    [Header("Debug")]
    public bool overrideAction;


    private void Update() {
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
    public abstract TStateMsg ResetAgent(TResetMsg resetAction);


    // Implement IAgent
    List<ISensor> IAgent.Sensors => _sensors;
    List<IActuator> IAgent.StateActuators => _stateActuators;
    List<IActuator> IAgent.ResetActuators => _resetActuators;

    void IAgent.Initialize() => Initialize();
    void IAgent.Action(Message action) => Action((TActionMsg)action);
    Message IAgent.State() => State();
    Message IAgent.ResetAgent(Message resetAction) => ResetAgent((TResetMsg)resetAction);



}