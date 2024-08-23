using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.InterfacesPkg;
using System;


using ActionMsg = RosMessageTypes.InterfacesPkg.AutonomousNavigationExampleAgentActionMsg;
using StateMsg = RosMessageTypes.InterfacesPkg.AutonomousNavigationExampleAgentStateMsg;
using ResetMsg = RosMessageTypes.InterfacesPkg.AutonomousNavigationExampleAgentResetMsg;


public class AutonomousNavigationExampleAgent : Agent<
    ActionMsg,
    StateMsg,
    ResetMsg> {

    public GameObject environment;

    [Header("Sensors")]
    [SerializeField] private PoseSensor _poseSensor;
    [SerializeField] private TwistSensor _twistSensor;
    [SerializeField] private PoseSensor _targetPoseSensor;
    [SerializeField] private LidarSensor _lidarSensor;
    [SerializeField] private SmartLidarSensor _smartLidarSensor;
    [SerializeField] private TriggerSensor _collisionTriggerSensor;
    [SerializeField] private TriggerSensor _targetTriggerSensor;

    [Header("Actuators")]
    [SerializeField] private TwistActuator _twistActuator;
    [SerializeField] private PoseActuator _poseActuator;
    [SerializeField] private PoseActuator _targetPoseActuator;


    public override void Initialize() {
        
        // Populate sensors list
        _sensors = new List<ISensor> {
            _poseSensor,
            _twistSensor,
            _targetPoseSensor,
            _lidarSensor,
            _smartLidarSensor,
            _collisionTriggerSensor,
            _targetTriggerSensor
        };

        // Populate state actuators list
        _stateActuators = new List<IActuator> {
            _twistActuator
        };

        // Populate reset actuators list
        _resetActuators = new List<IActuator> {
            _poseActuator,
            _targetPoseActuator
        };

        // Initialize sensors
        foreach (ISensor sensor in _sensors) {
            sensor.Initialize();
        }

        // Initialize state actuators
        foreach (IActuator actuator in _stateActuators) {
            actuator.Initialize();
        }

        // Initialize reset actuators
        foreach (IActuator actuator in _resetActuators) {
            actuator.Initialize();
        }
    }

    public override void Action(ActionMsg action) {

        // Set actuator data
        _twistActuator.SetActuatorData(action.twist);
        
    }

    public override StateMsg State() {

        // Get sensor data
        foreach (Sensor sensor in _sensors) {
            sensor.GetSensorData();
        }

        // Fill the response
        StateMsg state = new StateMsg {
            pose = _poseSensor.pose,
            target_pose = _targetPoseSensor.pose,
            twist = _twistSensor.twist,
            // laser_scan = _lidarSensor.laserScan,
            smart_lidar_sensor = _smartLidarSensor.smartLidarSensorMsg,
            collision_trigger_sensor = _collisionTriggerSensor.triggerSensorMsg,
            target_trigger_sensor = _targetTriggerSensor.triggerSensorMsg
        };

        return state;
    }

    public override StateMsg ResetAgent(ResetMsg resetAction) {
        
        // Reset sensors
        foreach (Sensor sensor in _sensors) {
            sensor.ResetSensor();
        }
        
        // Reset actuators
        _twistActuator.ResetActuator();
        _poseActuator.SetActuatorData(resetAction.agent_target_pose);
        _targetPoseActuator.SetActuatorData(resetAction.target_target_pose);

        // Return the state
        return State();
        
    }
}
