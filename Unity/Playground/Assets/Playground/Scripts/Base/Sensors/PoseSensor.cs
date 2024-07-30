using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Geometry;

public class PoseSensor : Sensor {

    [HideInInspector]
    public PoseMsg pose;

    public override void GetData() {

        // Convert Unity data to ROS message
        pose.position.x = transform.position.x;
        pose.position.y = transform.position.y;
        pose.position.z = transform.position.z;

        pose.orientation.x = transform.rotation.x;
        pose.orientation.y = transform.rotation.y;
        pose.orientation.z = transform.rotation.z;
        pose.orientation.w = transform.rotation.w;

    }

    public override void ResetSensor() {
    }
}
