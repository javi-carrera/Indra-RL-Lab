using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.InterfacesPkg;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

using ActionMsg = RosMessageTypes.InterfacesPkg.PoseSensorAndActuatorTestAgentActionMsg;
using StateMsg = RosMessageTypes.InterfacesPkg.PoseSensorAndActuatorTestAgentStateMsg;
using ResetMsg = RosMessageTypes.InterfacesPkg.PoseSensorAndActuatorTestAgentStateMsg;


public class PoseSensorAndActuatorTestAgent : Agent<
    ActionMsg,
    StateMsg,
    ResetMsg> {

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

    public override void Action(ActionMsg action) {
        // Set actuator data
        _poseActuator.SetData(action.target_pose);
    }

    public override StateMsg State() {

        // Get sensor data
        foreach (Sensor sensor in _sensors) {
            sensor.GetData();
        }

        // Fill the response
        StateMsg state = new StateMsg {
            pose = _poseSensor.pose
        };

        return state;
    }

    public override StateMsg ResetAgent(ResetMsg resetAction) {
        return State();
    }
}

