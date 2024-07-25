using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.InterfacesPkg;
using System;
using Unity.VisualScripting;

public class AgentExample : Agent {

    public PositionSensor _positionSensor;
    public PositionActuator _positionActuator;


    void Start() {
        
        // Get sensors and actuators
        // _positionSensor = GetComponent<PositionSensor>();
        // _positionActuator = GetComponent<PositionActuator>();

        _sensors = new List<Sensor> {
            _positionSensor,
        };
    }

    public override void SetActuatorDataFromAction(AgentActionMsg action) {
        
        // Set actuator data
        _positionActuator.SetData(action.target_position);
    }

    public override AgentStateMsg GetStateFromSensorData() {

        // Get sensor data
        foreach (Sensor sensor in _sensors) {
            sensor.GetData();
        }

        // Fill the response
        AgentStateMsg state = new AgentStateMsg();

        state.position = _positionSensor.position;

        return state;
    }

    public override AgentStateMsg ResetAgent() {

        Debug.Log("Resetting environment");

        // Reset environment logic here
        AgentStateMsg state = new AgentStateMsg();

        transform.position = Vector3.zero;

        return state;

    }


}
