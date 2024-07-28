using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.InterfacesPkg;

public class EscoPilotExampleAgent : Agent<EscoPilotExampleAgentActionMsg, EscoPilotExampleAgentStateMsg> {

    /*
    sensor_msgs/LaserScan laser_scan
    geometry_msgs/Pose pose
    geometry_msgs/Pose target_pose
    TriggerSensor target_trigger_sensor

    */

    // Sensors

    [SerializeField]
    private LidarSensor _lidarSensor;
    [SerializeField]
    private PoseSensor _poseSensor;
    [SerializeField]
    private PoseSensor _targetPoseSensor;
    [SerializeField]
    private TriggerSensor _targetTriggerSensor;

    // Actuators
    [SerializeField]
    private TwistActuator _twistActuator;

    void Start() {
        
        // Initialize sensors list
        _sensors = new List<Sensor> {
            _lidarSensor,
            _poseSensor,
            _targetPoseSensor,
            _targetTriggerSensor
        };
    }

    public override void PerformAction(EscoPilotExampleAgentActionMsg action) {
        // Set actuator data
        Debug.Log(action);
        _twistActuator.SetData(action.target_velocity);
    }

    public override EscoPilotExampleAgentStateMsg UpdateAgentState() {

        // Get sensor data
        foreach (Sensor sensor in _sensors) {
            sensor.GetData();
        }

        // Fill the response
        EscoPilotExampleAgentStateMsg state = new EscoPilotExampleAgentStateMsg{
            laser_scan = _lidarSensor.laserScan,
            pose = _poseSensor.pose,
            target_pose = _targetPoseSensor.pose,
            target_trigger_sensor = _targetTriggerSensor.triggerSensorMsg,


        };

        return state;
    }

    public override EscoPilotExampleAgentStateMsg ResetAgent() {
        return new EscoPilotExampleAgentStateMsg();
    }
}
