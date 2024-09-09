using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.InterfacesPkg;
using System;


using ActionMsg = RosMessageTypes.InterfacesPkg.ShootingExampleAgentActionMsg;
using StateMsg = RosMessageTypes.InterfacesPkg.ShootingExampleAgentStateMsg;


public class ShootingExampleAgent : Agent<
    ActionMsg,
    StateMsg> {
    
    public float maxLinearVelocity;
    public float maxAngularVelocity;

    [Header("Sensors")]
    [SerializeField] private PoseSensor _poseSensor;
    [SerializeField] private TwistSensor _twistSensor;
    [SerializeField] private SmartLidarSensor _smartLidarSensor;
    [SerializeField] private HealthSensor _healthSensor;

    [Header("Actuators")]
    [SerializeField] private TwistActuator _twistActuator;
    [SerializeField] private Turret2DActuator _turret2DActuator;


    public override void Initialize() {
        
        // Populate sensors list
        _sensors = new List<ISensor> {
            _poseSensor,
            _twistSensor,
            _smartLidarSensor,
            _healthSensor
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
        _twistActuator.SetActuatorData(action.twist_actuator);
        
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
            smart_lidar_sensor = _smartLidarSensor.smartLidarSensorMsg,
            health_sensor = _healthSensor.healthSensorMsg,
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
