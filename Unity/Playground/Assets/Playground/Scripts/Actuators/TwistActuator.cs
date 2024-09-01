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

    [HideInInspector] public Vector3 targetLinearVelocity;
    [HideInInspector] public Vector3 targetAngularVelocity;
    private Vector3 _currentLinearVelocity;
    private Vector3 _currentAngularVelocity;


    [Header("Debug Settings")]
    public bool drawDebugLines;


    public override void Initialize() {

        // Get the rigidbody of the target object
        _rb = target.GetComponent<Rigidbody>();

        _currentLinearVelocity = target.transform.InverseTransformDirection(_rb.velocity);
        _currentAngularVelocity = target.transform.InverseTransformDirection(_rb.angularVelocity);

        // Reset the actuator
        ResetActuator();
    }


    public override void SetActuatorData(TwistMsg msg) {

        // Convert ROS pose message to Unity data
        targetLinearVelocity = PoseConverter.Ros2UnityLinearVelocity(msg.linear);
        targetAngularVelocity = PoseConverter.Ros2UnityAngularVelocity(msg.angular);
    }

    public override void ResetActuator() {

        // Reset the current and target velocities to zero
        _currentLinearVelocity = Vector3.zero;
        _currentAngularVelocity = Vector3.zero;
        targetLinearVelocity = Vector3.zero;
        targetAngularVelocity = Vector3.zero;

        // Reset the rigidbody of the target object
        _rb.velocity = Vector3.zero;
        _rb.angularVelocity = Vector3.zero;

    }

    protected override void UpdateActuator() {

        if (instantTwist) {
            // Apply the target velocities to the target object's rigidbody
            _rb.velocity = target.transform.TransformDirection(targetLinearVelocity);
            _rb.angularVelocity = target.transform.TransformDirection(targetAngularVelocity);
            
        }

        else {


            // Lerp the target velocities
            _currentLinearVelocity = Vector3.Lerp(_currentLinearVelocity, targetLinearVelocity, linearTimeConstant * Time.deltaTime);
            _currentAngularVelocity = Vector3.Lerp(_currentAngularVelocity, targetAngularVelocity, angularTimeConstant * Time.deltaTime);

            // Transform the target velocities to the target object's world space and apply the target velocities to the target object's rigidbody
            _rb.velocity = target.transform.TransformDirection(_currentLinearVelocity);
            _rb.angularVelocity = target.transform.TransformDirection(_currentAngularVelocity);
        }

        // Draw debug lines
        if (drawDebugLines) {
            DrawDebugLines();
        }
    }


    private void DrawDebugLines() {
        Debug.DrawLine(target.transform.position, target.transform.position + _rb.velocity, Color.blue);
        Debug.DrawLine(target.transform.position, target.transform.position + _rb.angularVelocity, Color.red);
    }

}
