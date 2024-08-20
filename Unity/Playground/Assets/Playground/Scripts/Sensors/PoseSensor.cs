using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Geometry;

public class PoseSensor : Sensor {

    [HideInInspector] public PoseMsg pose;

    [Header("Pose Sensor Settings")]
    public GameObject target;
    [HideInInspector] public Vector3 environmentPosition;

    public override void Initialize() {
        pose = new PoseMsg();
    }
    
    public override void GetSensorData() {

        // Convert Unity data to ROS message
        pose.position.x = target.transform.position.x - environmentPosition.x;
        pose.position.y = target.transform.position.y - environmentPosition.y;
        pose.position.z = target.transform.position.z - environmentPosition.z;

        pose.orientation.x = target.transform.rotation.x;
        pose.orientation.y = target.transform.rotation.y;
        pose.orientation.z = target.transform.rotation.z;
        pose.orientation.w = target.transform.rotation.w;

    }

    protected override void UpdateSensor() {
    }

    public override void ResetSensor() {
    }
}
