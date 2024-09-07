using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.InterfacesPkg;
using System;


using ActionMsg = RosMessageTypes.InterfacesPkg.ShootingExampleAgentActionMsg;
using StateMsg = RosMessageTypes.InterfacesPkg.ShootingExampleAgentStateMsg;
using ResetMsg = RosMessageTypes.InterfacesPkg.ShootingExampleAgentResetMsg;


public class ShootingExampleAgent : Agent<
    ActionMsg,
    StateMsg,
    ResetMsg> {
    
    public float maxLinearVelocity;
    public float maxAngularVelocity;

    [Header("Sensors")]
    [SerializeField] private PoseSensor _poseSensor;
    [SerializeField] private TwistSensor _twistSensor;
    [SerializeField] private SmartLidarSensor _smartLidarSensor;
    [SerializeField] private TriggerSensor _collisionTriggerSensor;

    [Header("Actuators")]
    [SerializeField] private TwistActuator _twistActuator;
    [SerializeField] private Turret2DActuator _turret2DActuator;
    [SerializeField] private PoseActuator _poseActuator;


    public override void Initialize() {
        
        // Populate sensors list
        _sensors = new List<ISensor> {
            _poseSensor,
            _twistSensor,
            _smartLidarSensor,
            _collisionTriggerSensor,
        };

        // Populate state actuators list
        _stateActuators = new List<IActuator> {
            _twistActuator,
            _turret2DActuator,
        };

        // Populate reset actuators list
        _resetActuators = new List<IActuator> {
            _poseActuator,
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


    public override void OverrideAction() {
        
        // Override linear and angular velocity
        Vector3 overridenLinearVelocity = maxLinearVelocity * new Vector3(Input.GetAxis("Vertical"), 0, 0 );
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
            twist = _twistSensor.twist,
            smart_lidar_sensor = _smartLidarSensor.smartLidarSensorMsg,
            collision_trigger_sensor = _collisionTriggerSensor.triggerSensorMsg,
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

        // Return the state
        return State();
        
    }
}
