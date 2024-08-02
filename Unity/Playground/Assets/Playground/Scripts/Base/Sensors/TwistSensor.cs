using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Geometry;

public class TwistSensor : Sensor {

    public GameObject target;
    [HideInInspector]
    public TwistMsg twist;

    private Rigidbody _rb;

    void Start() {
        twist = new TwistMsg();
        _rb = target.GetComponent<Rigidbody>();
    }

    public override void GetData() {

        Vector3 linearVelocity = _rb.velocity;
        Vector3 angularVelocity = _rb.angularVelocity;

        // Transform the velocities to the target object's local frame
        linearVelocity = target.transform.InverseTransformDirection(linearVelocity);
        angularVelocity = target.transform.InverseTransformDirection(angularVelocity);

        // Convert Unity data to ROS message
        
        twist.linear.x = linearVelocity.x;
        twist.linear.y = linearVelocity.y;
        twist.linear.z = linearVelocity.z;

        twist.angular.x = angularVelocity.x;
        twist.angular.y = angularVelocity.y;
        twist.angular.z = angularVelocity.z;

    }

    public override void ResetSensor() {
    }
}
