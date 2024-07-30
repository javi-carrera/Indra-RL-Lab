using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.InterfacesPkg;

public class PoseSensorAndActuatorTestAgent : Agent<PoseSensorAndActuatorTestAgentActionMsg, PoseSensorAndActuatorTestAgentStateMsg> {

    [SerializeField]
    private PoseSensor _poseSensor;

    [SerializeField]
    private PoseActuator _poseActuator;


    void Start() {
        
        // Initialize sensors list
        _sensors = new List<Sensor> {
            _poseSensor,
        };
    }

    public override void PerformAction(PoseSensorAndActuatorTestAgentActionMsg action) {
        // Set actuator data
        _poseActuator.SetData(action.target_pose);
    }

    public override PoseSensorAndActuatorTestAgentStateMsg UpdateAgentState() {

        // Get sensor data
        foreach (Sensor sensor in _sensors) {
            sensor.GetData();
        }

        // Fill the response
        PoseSensorAndActuatorTestAgentStateMsg state = new PoseSensorAndActuatorTestAgentStateMsg {
            pose = _poseSensor.pose
        };

        return state;
    }

    public override void ResetAgent(PoseSensorAndActuatorTestAgentActionMsg resetAction) {
    }
}
