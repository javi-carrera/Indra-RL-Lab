using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.InterfacesPkg;
using System;


using ActionMsg = RosMessageTypes.InterfacesPkg.UC2AgentActionMsg;
using StateMsg = RosMessageTypes.InterfacesPkg.UC2AgentStateMsg;


public class UC2Agent : Agent<
    ActionMsg,
    StateMsg> {
    
    public float maxLinearVelocity;
    public float maxAngularVelocity;

    [Header("Tank Sensors")]
    [SerializeField] private Pose2DSensor _pose2DSensor;
    [SerializeField] private Twist2DSensor _twist2DSensor;
    [SerializeField] private HealthSensor _healthSensor;
    [SerializeField] private SmartLidar2DSensor _smartLidar2DSensor;
    [SerializeField] private Turret2DSensor _turret2DSensor;

    [Header("Target Sensors")]
    [SerializeField] private Pose2DSensor _targetPose2DSensor;
    [SerializeField] private HealthSensor _targetHealthSensor;

    [Header("Tank Actuators")]
    [SerializeField] private Twist2DActuator _twistActuator;
    [SerializeField] private Turret2DActuator _turret2DActuator;


    public override void Initialize() {
        
        // Populate sensors list
        _sensors = new List<ISensor> {
            _pose2DSensor,
            _twist2DSensor,
            _healthSensor,
            _smartLidar2DSensor,
            _turret2DSensor,
            _targetPose2DSensor,
            _targetHealthSensor,
        };

        // Populate state actuators list
        _actuators = new List<IActuator> {
            _twistActuator,
            _turret2DActuator,
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
        
        // Override linear and angular velocity
        Vector3 overridenLinearVelocity = maxLinearVelocity * new Vector3(0, 0, Input.GetAxis("Vertical"));
        Vector3 overridenAngularVelocity = maxAngularVelocity * new Vector3(0, Input.GetAxis("Horizontal"), 0);

        _twistActuator.targetLinearVelocity = overridenLinearVelocity;
        _twistActuator.targetAngularVelocity = overridenAngularVelocity;

        // Override turret angle (controlled with mouse scroll wheel)
        _turret2DActuator.targetAngle += Input.GetAxis("Mouse ScrollWheel") * _turret2DActuator.rotationSpeed;


        // Override fire
        if (Input.GetButton("Fire1")) {
            _turret2DActuator.fire = true;
        }
        else {
            _turret2DActuator.fire = false;
        }

    }


    public override void Action(ActionMsg action) {

        if (overrideAction) return;

        // Set actuator data
        _twistActuator.SetActuatorData(action.tank.target_twist);
        _turret2DActuator.SetActuatorData(action.tank.turret_actuator);
        
    }

    public override StateMsg State() {

        // Get sensor data
        foreach (ISensor sensor in _sensors) {
            sensor.GetSensorData();
        }

        TankStateMsg tankStateMsg = new TankStateMsg {
            pose = _pose2DSensor.pose2DMsg,
            twist = _twist2DSensor.twist2DMsg,
            smart_laser_scan = _smartLidar2DSensor.smartLaserScan2DMsg,
            health_info = _healthSensor.healthInfoMsg,
            turret_sensor = _turret2DSensor.turret2DSensorMsg,
        };

        // Fill the response
        StateMsg state = new StateMsg {
            tank_state = tankStateMsg,
            target_pose = _targetPose2DSensor.pose2DMsg,
            target_health_info = _targetHealthSensor.healthInfoMsg,
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
