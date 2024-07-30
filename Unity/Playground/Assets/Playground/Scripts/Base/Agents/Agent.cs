using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.InterfacesPkg;
using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;


public abstract class Agent<TAgentActionMsg, TAgentStateMsg> : MonoBehaviour where TAgentActionMsg : Message, new() where TAgentStateMsg : Message, new() {

    protected List<Sensor> _sensors;
    protected TAgentStateMsg _state;

    public abstract void PerformAction(TAgentActionMsg action);

    public abstract TAgentStateMsg UpdateAgentState();

    public abstract void ResetAgent(TAgentActionMsg resetAction);


}
