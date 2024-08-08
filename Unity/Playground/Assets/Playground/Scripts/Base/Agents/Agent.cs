using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.InterfacesPkg;
using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;


public abstract class Agent<TActionMsg, TStateMsg, TResetMsg> : MonoBehaviour
    where TActionMsg : Message, new()
    where TStateMsg : Message, new()
    where TResetMsg : Message, new() {

    
    protected List<ISensor> _sensors;
    protected List<IActuator> _stateActuators;
    protected List<IActuator> _resetActuators;


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

}