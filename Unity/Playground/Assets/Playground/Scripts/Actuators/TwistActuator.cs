using System.Collections;
using System.Collections.Generic;
using RosMessageTypes.Geometry;
using UnityEngine;

public class TwistActuator : Actuator<TwistMsg> {

    public bool overrideAction;
    public float linearVelocity;
    public float angularVelocity;

    public GameObject target;
    public float maxLinearVelocity;
    public float maxAngularVelocity;
    public float maxLinearAcceleration;
    public float maxAngularAcceleration;
    private Vector3 _targetLinearVelocity;
    private Vector3 _targetAngularVelocity;
    private Vector3 _targetLinearVelocity_world;
    private Vector3 _targetAngularVelocity_world;


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

    void Update() {
        // Apply the target velocities to the target object
        Rigidbody rb = target.GetComponent<Rigidbody>();

        if (overrideAction) {
            _targetLinearVelocity = new Vector3(linearVelocity, 0, 0);
            _targetAngularVelocity = new Vector3(0, angularVelocity, 0);
        }


        // Convert the target velocities to the target object's local space
        _targetLinearVelocity_world = target.transform.TransformDirection(_targetLinearVelocity);
        _targetAngularVelocity_world = target.transform.TransformDirection(_targetAngularVelocity);

        // Lerp the velocities
        
        // rb.velocity = Vector3.Lerp(rb.velocity, _targetLinearVelocity, maxLinearAcceleration * Time.deltaTime);
        // rb.angularVelocity = Vector3.Lerp(rb.angularVelocity, _targetAngularVelocity, maxAngularAcceleration * Time.deltaTime);
        rb.velocity = _targetLinearVelocity_world;
        rb.angularVelocity = _targetAngularVelocity_world;

        // print rb velocity in local coords
        Debug.Log($"rb local: {target.transform.InverseTransformDirection(rb.velocity)}");

    }
}
