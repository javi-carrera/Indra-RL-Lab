using System.Collections;
using System.Collections.Generic;
using RosMessageTypes.Geometry;
using UnityEngine;

public class TwistActuator : Actuator<TwistMsg> {

    public GameObject target;
    public float linearTimeConstant;    // [s]
    public float angularTimeConstant;   // [s]
    public bool drawDebugLines;
    public bool overrideAction;
    public Vector3 overridenLinearVelocity;
    public Vector3 overridenAngularVelocity;

    private Rigidbody _rb;
    private Vector3 _targetLinearVelocity;
    private Vector3 _targetAngularVelocity;

    void Start() {

        // Get the rigidbody of the target object
        _rb = target.GetComponent<Rigidbody>();

        // Reset the actuator
        ResetActuator();
    }


    public override void SetData(TwistMsg msg) {

        // Convert ROS pose message to Unity data

        _targetLinearVelocity = new Vector3(
            (float)msg.linear.x,
            (float)msg.linear.y,
            (float)msg.linear.z
        );

        _targetAngularVelocity = new Vector3(
            (float)msg.angular.x,
            (float)msg.angular.y,
            (float)msg.angular.z
        );
    }

    public override void ResetActuator() {

        // Reset the target velocities to zero
        _targetLinearVelocity = Vector3.zero;
        _targetAngularVelocity = Vector3.zero;

        // Reset the rigidbody of the target object
        _rb.velocity = Vector3.zero;
        _rb.angularVelocity = Vector3.zero;

    }

    void Update() {

        Vector3 _targetLinearVelocityWorld;
        Vector3 _targetAngularVelocityWorld;

        // Convert the target velocities to the target object's local space
        if (overrideAction) {
            _targetLinearVelocityWorld = target.transform.TransformDirection(overridenLinearVelocity);
            _targetAngularVelocityWorld = target.transform.TransformDirection(overridenAngularVelocity);
        }
        else {
            _targetLinearVelocityWorld = target.transform.TransformDirection(_targetLinearVelocity);
            _targetAngularVelocityWorld = target.transform.TransformDirection(_targetAngularVelocity);
        }

        // Lerp the velocities to the target velocities with the given time constants
        _rb.velocity = Vector3.Lerp(_rb.velocity, _targetLinearVelocityWorld, linearTimeConstant * Time.deltaTime);
        _rb.angularVelocity = Vector3.Lerp(_rb.angularVelocity, _targetAngularVelocityWorld, angularTimeConstant * Time.deltaTime);

        // Draw debug lines
        if (drawDebugLines) {
            Debug.DrawLine(target.transform.position, target.transform.position + _rb.velocity, Color.blue);
            Debug.DrawLine(target.transform.position, target.transform.position + _rb.angularVelocity, Color.red);
        }

    }
}
