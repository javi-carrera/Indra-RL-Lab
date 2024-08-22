using System.Collections;
using System.Collections.Generic;
using RosMessageTypes.Geometry;
using UnityEngine;


public class TwistActuator : Actuator<TwistMsg> {

    [Header("Twist Actuator Settings")]
    public GameObject target;
    public bool instantTwist;
    public float linearTimeConstant;
    public float angularTimeConstant;
    private Rigidbody _rb;
    private Vector3 _targetLinearVelocity;
    private Vector3 _targetAngularVelocity;

    [Header("Debug Settings")]
    public bool drawDebugLines;
    public bool overrideAction;
    public Vector3 overridenLinearVelocity;
    public Vector3 overridenAngularVelocity;


    public override void Initialize() {

        // Get the rigidbody of the target object
        _rb = target.GetComponent<Rigidbody>();

        // Reset the actuator
        ResetActuator();
    }


    public override void SetActuatorData(TwistMsg msg) {

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

    protected override void UpdateActuator() {

        Vector3 targetLinearVelocity;
        Vector3 targetAngularVelocity;
        Vector3 targetLinearVelocityLerped;
        Vector3 targetAngularVelocityLerped;
        Vector3 targetLinearVelocityWorld;
        Vector3 targetAngularVelocityWorld;


        // Set the target velocities based on the overrideAction flag
        if (overrideAction) {
            targetLinearVelocity = overridenLinearVelocity;
            targetAngularVelocity = overridenAngularVelocity;
        }
        else {
            targetLinearVelocity = _targetLinearVelocity;
            targetAngularVelocity = _targetAngularVelocity;
        }

        if (instantTwist) {

            // Apply the target velocities to the target object's rigidbody
            _rb.velocity = target.transform.TransformDirection(targetLinearVelocity);
            _rb.angularVelocity = target.transform.TransformDirection(targetAngularVelocity);
            
        }

        else {

            // Lerp the target velocities (component-wise)
            targetLinearVelocityLerped = new Vector3(
                Mathf.Lerp(target.transform.InverseTransformDirection(_rb.velocity).x, targetLinearVelocity.x, linearTimeConstant * Time.deltaTime),
                Mathf.Lerp(target.transform.InverseTransformDirection(_rb.velocity).y, targetLinearVelocity.y, linearTimeConstant * Time.deltaTime),
                Mathf.Lerp(target.transform.InverseTransformDirection(_rb.velocity).z, targetLinearVelocity.z, linearTimeConstant * Time.deltaTime)
            );
            targetAngularVelocityLerped = new Vector3(
                Mathf.Lerp(target.transform.InverseTransformDirection(_rb.angularVelocity).x, targetAngularVelocity.x, angularTimeConstant * Time.deltaTime),
                Mathf.Lerp(target.transform.InverseTransformDirection(_rb.angularVelocity).y, targetAngularVelocity.y, angularTimeConstant * Time.deltaTime),
                Mathf.Lerp(target.transform.InverseTransformDirection(_rb.angularVelocity).z, targetAngularVelocity.z, angularTimeConstant * Time.deltaTime)
            );

            // Transform the target velocities to the target object's world space
            targetLinearVelocityWorld = target.transform.TransformDirection(targetLinearVelocityLerped);
            targetAngularVelocityWorld = target.transform.TransformDirection(targetAngularVelocityLerped);

            // Apply the target velocities to the target object's rigidbody
            _rb.velocity = targetLinearVelocityWorld;
            _rb.angularVelocity = targetAngularVelocityWorld;
        }

        // Draw debug lines
        if (drawDebugLines) {
            Debug.DrawLine(target.transform.position, target.transform.position + _rb.velocity, Color.blue);
            Debug.DrawLine(target.transform.position, target.transform.position + _rb.angularVelocity, Color.red);
        }
    }

}
