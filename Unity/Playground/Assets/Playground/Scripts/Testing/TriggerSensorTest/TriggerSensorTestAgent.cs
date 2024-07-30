using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.InterfacesPkg;

public class TriggerSensorTestAgent : Agent<TriggerSensorTestAgentActionMsg, TriggerSensorTestAgentStateMsg> {

    [SerializeField]
    private TriggerSensor _triggerSensor01;
    [SerializeField]
    private TriggerSensor _triggerSensor02;
    [SerializeField]
    private TriggerSensor _triggerSensor03;
    [SerializeField]
    private TriggerSensor _triggerSensor04;


    void Start() {
        
        // Initialize sensors list
        _sensors = new List<Sensor> {
            _triggerSensor01,
            _triggerSensor02,
            _triggerSensor03,
            _triggerSensor04
        };
    }

    public override void PerformAction(TriggerSensorTestAgentActionMsg action) {
        // Set actuator data
    }

    public override TriggerSensorTestAgentStateMsg UpdateAgentState() {

        // Get sensor data
        foreach (Sensor sensor in _sensors) {
            sensor.GetData();
        }

        // Fill the response
        TriggerSensorTestAgentStateMsg state = new TriggerSensorTestAgentStateMsg {
            trigger_sensor_01_data = _triggerSensor01.triggerSensorMsg,
            trigger_sensor_02_data = _triggerSensor02.triggerSensorMsg,
            trigger_sensor_03_data = _triggerSensor03.triggerSensorMsg,
            trigger_sensor_04_data = _triggerSensor04.triggerSensorMsg
        };

        return state;
    }

    public override void ResetAgent(TriggerSensorTestAgentActionMsg resetAction) {
    }
}
