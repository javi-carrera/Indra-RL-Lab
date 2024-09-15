using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.InterfacesPkg;
using System;


using ActionMsg = RosMessageTypes.InterfacesPkg.UC1AgentActionMsg;
using StateMsg = RosMessageTypes.InterfacesPkg.UC1AgentStateMsg;


public class UC1Agent : Agent<
    ActionMsg,
    StateMsg> {

    public float maxLinearVelocity;
    public float maxAngularVelocity;

    [Header("Tank Sensors")]
    [SerializeField] private Pose2DSensor _pose2DSensor;
    [SerializeField] private Twist2DSensor _twist2DSensor;
    [SerializeField] private HealthSensor _healthSensor;
    [SerializeField] private SmartLidar2DSensor _smartLidar2DSensor;

    [Header("Target Sensors")]
    [SerializeField] private Pose2DSensor _targetPose2DSensor;
    [SerializeField] private TriggerSensor _targetTriggerSensor;

    [Header("Tank Actuators")]
    [SerializeField] private Twist2DActuator _twist2DActuator;


    public override void Initialize() {
        
        // Populate sensors list
        _sensors = new List<ISensor> {
            _pose2DSensor,
            _twist2DSensor,
            _smartLidar2DSensor,
            _healthSensor,
            _targetPose2DSensor,
            _targetTriggerSensor
        };

        // Populate state actuators list
        _actuators = new List<IActuator> {
            _twist2DActuator
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

        _twist2DActuator.targetLinearVelocity = overridenLinearVelocity;
        _twist2DActuator.targetAngularVelocity = overridenAngularVelocity;

    }


    public override void Action(ActionMsg action) {

        if (overrideAction) return;

        // Set actuator data
        _twist2DActuator.SetActuatorData(action.tank_action.target_twist2d);
        
    }

    public override StateMsg State() {

        // Get sensor data
        foreach (ISensor sensor in _sensors) {
            sensor.GetSensorData();
        }

        // Fill the response
        TankStateMsg tankStateMsg = new TankStateMsg {
            pose2d = _pose2DSensor.pose2DMsg,
            twist2d = _twist2DSensor.twist2DMsg,
            smart_laser_scan2d = _smartLidar2DSensor.smartLaserScan2DMsg,
            health_info = _healthSensor.healthInfoMsg
        };

        StateMsg state = new StateMsg {
            tank_state = tankStateMsg,
            target_pose2d = _targetPose2DSensor.pose2DMsg,
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
        _twist2DActuator.ResetActuator();

        // Return the state
        return State();
        
    }
}
