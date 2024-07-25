using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.InterfacesPkg;
using System;
using Unity.VisualScripting;

public abstract class Agent : MonoBehaviour {

    protected List<Sensor> _sensors;
    protected AgentStateMsg _state;


    void Start() {
    }    

    public abstract void SetActuatorDataFromAction(AgentActionMsg action);

    public abstract AgentStateMsg GetStateFromSensorData();

    public abstract AgentStateMsg ResetAgent();


}
