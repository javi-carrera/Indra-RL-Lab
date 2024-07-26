using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.InterfacesPkg;

public class LidarSensorTestAgent : Agent<LidarSensorTestAgentActionMsg, LidarSensorTestAgentStateMsg> {

    public LidarSensor _lidarSensor;


    void Start() {
        
        // Initialize sensors list
        _sensors = new List<Sensor> {
            _lidarSensor,
        };
    }

    public override void PerformAction(LidarSensorTestAgentActionMsg action) {
        // Set actuator data
    }

    public override LidarSensorTestAgentStateMsg UpdateAgentState() {

        // Get sensor data
        foreach (Sensor sensor in _sensors) {
            sensor.GetData();
        }

        // Fill the response
        LidarSensorTestAgentStateMsg state = new LidarSensorTestAgentStateMsg();

        state.laser_scan = _lidarSensor.laserScan;

        return state;
    }

    public override LidarSensorTestAgentStateMsg ResetAgent() {
        return new LidarSensorTestAgentStateMsg();
    }
}
