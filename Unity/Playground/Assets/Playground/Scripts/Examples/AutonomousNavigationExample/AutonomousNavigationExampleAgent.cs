using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.InterfacesPkg;
using System;


using ActionMsg = RosMessageTypes.InterfacesPkg.AutonomousNavigationExampleAgentActionMsg;
using StateMsg = RosMessageTypes.InterfacesPkg.AutonomousNavigationExampleAgentStateMsg;


public class AutonomousNavigationExampleAgent : Agent<
    ActionMsg,
    StateMsg> {

    public float maxLinearVelocity;
    public float maxAngularVelocity;

    [Header("Sensors")]
    [SerializeField] private PoseSensor _poseSensor;
    [SerializeField] private TwistSensor _twistSensor;
    [SerializeField] private LidarSensor _lidarSensor;
    [SerializeField] private HealthSensor _healthSensor;
    [SerializeField] private PoseSensor _targetPoseSensor;
    [SerializeField] private TriggerSensor _targetTriggerSensor;

    [Header("Actuators")]
    [SerializeField] private TwistActuator _twistActuator;


    public override void Initialize() {
        
        // Populate sensors list
        _sensors = new List<ISensor> {
            _poseSensor,
            _twistSensor,
            _lidarSensor,
            _healthSensor,
            _targetPoseSensor,
            _targetTriggerSensor
        };

        // Populate state actuators list
        _actuators = new List<IActuator> {
            _twistActuator
        };

        // Initialize sensors
        foreach (ISensor sensor in _sensors) {
            sensor.Initialize();
        }

        // Initialize state actuators
        foreach (IActuator actuator in _actuators) {
            actuator.Initialize();
        }

    }


    public override void OverrideAction() {
        
        Vector3 overridenLinearVelocity = maxLinearVelocity * new Vector3(0, 0, Input.GetAxis("Vertical"));
        Vector3 overridenAngularVelocity = maxAngularVelocity * new Vector3(0, Input.GetAxis("Horizontal"), 0);

        _twistActuator.targetLinearVelocity = overridenLinearVelocity;
        _twistActuator.targetAngularVelocity = overridenAngularVelocity;

    }


    public override void Action(ActionMsg action) {

        if (overrideAction) return;

        // Set actuator data
        _twistActuator.SetActuatorData(action.twist);
        
    }

    public override StateMsg State() {

        // Get sensor data
        foreach (ISensor sensor in _sensors) {
            sensor.GetSensorData();
        }

        // Fill the response
        StateMsg state = new StateMsg {
            pose_sensor = _poseSensor.pose,
            twist_sensor = _twistSensor.twist,
            lidar_sensor = _lidarSensor.laserScan,
            health_sensor = _healthSensor.healthSensorMsg,
            target_pose_sensor = _targetPoseSensor.pose,
            target_trigger_sensor = _targetTriggerSensor.triggerSensorMsg
        };

        return state;
    }

    public override StateMsg ResetAgent() {
        
        // Reset sensors
        foreach (ISensor sensor in _sensors) {
            sensor.ResetSensor();
        }
        
        // Reset actuators
        _twistActuator.ResetActuator();

        // Return the state
        return State();
        
    }
}
