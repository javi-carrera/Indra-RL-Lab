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
    [SerializeField] private PoseSensor _targetPoseSensor;
    [SerializeField] private LidarSensor _lidarSensor;
    [SerializeField] private TriggerSensor _collisionTriggerSensor;
    [SerializeField] private TriggerSensor _targetTriggerSensor;

    [Header("Actuators")]
    [SerializeField] private TwistActuator _twistActuator;
    [SerializeField] private PoseActuator _poseActuator;
    [SerializeField] private PoseActuator _targetPoseActuator;


    public override void Initialize() {
        
        // Initialize state sensors list
        _sensors = new List<ISensor> {
            _poseSensor,
            _targetPoseSensor,
            _lidarSensor,
            _collisionTriggerSensor,
            _targetTriggerSensor
        };

        // Initialize state actuators list
        _stateActuators = new List<IActuator> {
            _twistActuator
        };

        // Initialize reset actuators list
        _resetActuators = new List<IActuator> {
            _poseActuator,
            _targetPoseActuator
        };

        // Initialize sensors
        foreach (ISensor sensor in _sensors) {
            sensor.Initialize();
        }

        // Initialize actuators
        foreach (IActuator actuator in _stateActuators) {
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
            laser_scan = _lidarSensor.laserScan,
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
