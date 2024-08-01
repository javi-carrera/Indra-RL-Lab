using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.InterfacesPkg;


using ActionMsg = RosMessageTypes.InterfacesPkg.LidarSensorTestAgentActionMsg;
using StateMsg = RosMessageTypes.InterfacesPkg.LidarSensorTestAgentStateMsg;
using ResetMsg = RosMessageTypes.InterfacesPkg.LidarSensorTestAgentStateMsg;


public class LidarSensorTestAgent : Agent<
    ActionMsg,
    StateMsg,
    ResetMsg> {
    

    [SerializeField]
    private LidarSensor _lidarSensor;


    void Start() {
        
        // Initialize sensors list
        _sensors = new List<Sensor> {
            _lidarSensor,
        };
    }

    public override void Action(ActionMsg action) {
        // Set actuator data
    }

    public override StateMsg State() {

        // Get sensor data
        foreach (Sensor sensor in _sensors) {
            sensor.GetData();
        }

        // Fill the response
        StateMsg state = new StateMsg {
            laser_scan = _lidarSensor.laserScan
        };

        return state;
    }

    public override StateMsg ResetAgent(ResetMsg resetAction) {
        return new StateMsg();
    }
}
