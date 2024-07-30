using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.InterfacesPkg;

public class AutonomousNavigationExampleAgent : Agent<AutonomousNavigationExampleAgentActionMsg, AutonomousNavigationExampleAgentStateMsg> {

    [Header("Sensors")]
    [SerializeField]
    private PoseSensor _poseSensor;
    [SerializeField]
    private PoseSensor _targetPoseSensor;
    [SerializeField]
    private LidarSensor _lidarSensor;
    [SerializeField]
    private TriggerSensor _collisionTriggerSensor;
    [SerializeField]
    private TriggerSensor _targetTriggerSensor;

    [Header("Actuators")]
    [SerializeField]
    private TwistActuator _twistActuator;
    [SerializeField]
    private PoseActuator _poseActuator;
    [SerializeField]
    private PoseActuator _targetPoseActuator;


    void Start() {
        
        // Initialize sensors list
        _sensors = new List<Sensor> {
            _poseSensor,
            _targetPoseSensor,
            _lidarSensor,
            _collisionTriggerSensor,
            _targetTriggerSensor
        };
    }

    public override void PerformAction(AutonomousNavigationExampleAgentActionMsg action) {
        // Set actuator data
        _twistActuator.SetData(action.twist);
    }

    public override AutonomousNavigationExampleAgentStateMsg UpdateAgentState() {

        // Get sensor data
        foreach (Sensor sensor in _sensors) {
            sensor.GetData();
        }

        // Fill the response
        AutonomousNavigationExampleAgentStateMsg state = new AutonomousNavigationExampleAgentStateMsg {
            pose = _poseSensor.pose,
            target_pose = _targetPoseSensor.pose,
            laser_scan = _lidarSensor.laserScan,
            collision_trigger_sensor = _collisionTriggerSensor.triggerSensorMsg,
            target_trigger_sensor = _targetTriggerSensor.triggerSensorMsg
        };

        return state;
    }

    public override void ResetAgent(AutonomousNavigationExampleAgentActionMsg resetAction) {
        
        // Reset sensors
        foreach (Sensor sensor in _sensors) {
            sensor.ResetSensor();
        }
        
        // Reset actuators
        _twistActuator.ResetActuator();
        _poseActuator.SetData(resetAction.agent_target_pose);
        _targetPoseActuator.SetData(resetAction.target_target_pose);
    }
}
