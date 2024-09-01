using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Geometry;


public class TwistSensor : Sensor {

    [HideInInspector] public TwistMsg twist;

    [Header("Twist Sensor Settings")]
    public GameObject target;
    private Rigidbody _rb;

    public override void Initialize() {
        twist = new TwistMsg();
        _rb = target.GetComponent<Rigidbody>();
    }

    public override void GetSensorData() {

        Vector3 linearVelocity = _rb.velocity;
        Vector3 angularVelocity = _rb.angularVelocity;

        // Transform the velocities to the target object's local frame
        linearVelocity = target.transform.InverseTransformDirection(linearVelocity);
        angularVelocity = target.transform.InverseTransformDirection(angularVelocity);

        // Convert Unity data to ROS message
        twist.linear = PoseConverter.Unity2RosLinearVelocity(linearVelocity);
        twist.angular = PoseConverter.Unity2RosAngularVelocity(angularVelocity);

    }

    protected override void UpdateSensor() {
    }

    public override void ResetSensor() {
    }
}
