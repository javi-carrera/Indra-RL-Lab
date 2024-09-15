using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.InterfacesPkg;


public class Twist2DSensor : Sensor {

    [HideInInspector] public Twist2DMsg twist2DMsg;

    [Header("Twist Sensor Settings")]
    public GameObject target;
    private Rigidbody _rb;

    public override void Initialize() {
        twist2DMsg = new Twist2DMsg();
        _rb = target.GetComponent<Rigidbody>();
    }

    public override void GetSensorData() {

        Vector3 linearVelocity = _rb.velocity;
        Vector3 angularVelocity = _rb.angularVelocity;

        // Transform the velocities to the target object's local frame
        linearVelocity = target.transform.InverseTransformDirection(linearVelocity);
        angularVelocity = target.transform.InverseTransformDirection(angularVelocity);

        // Convert Unity data to ROS message
        twist2DMsg.x = linearVelocity.x;
        twist2DMsg.y = linearVelocity.z;
        twist2DMsg.theta = -angularVelocity.y;

    }

    protected override void UpdateSensor() {
    }

    public override void ResetSensor() {
    }
}
