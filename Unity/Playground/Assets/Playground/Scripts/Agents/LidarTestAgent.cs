using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.InterfacesPkg;

public class LidarTestAgent : Agent<LidarTestAgentActionMsg, LidarTestAgentStateMsg> {

    public LidarSensor _lidarSensor;


    void Start() {
        
        // Initialize sensors list
        _sensors = new List<Sensor> {
            _lidarSensor,
        };
    }

    public override void PerformAction(LidarTestAgentActionMsg action) {
        // Set actuator data
    }

    public override LidarTestAgentStateMsg UpdateAgentState() {

        // Get sensor data
        foreach (Sensor sensor in _sensors) {
            sensor.GetData();
        }

        // Fill the response
        LidarTestAgentStateMsg state = new LidarTestAgentStateMsg();

        state.laser_scan = _lidarSensor.laserScan;

        return state;
    }

    public override LidarTestAgentStateMsg ResetAgent() {

        Debug.Log("Resetting environment");

        // Reset environment logic here
        LidarTestAgentStateMsg state = new LidarTestAgentStateMsg();

        transform.position = Vector3.zero;

        return state;
    }
}
