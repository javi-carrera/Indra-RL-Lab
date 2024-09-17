using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.InterfacesPkg;

public class Pose2DSensor : Sensor {

    [HideInInspector] public Pose2DMsg pose2DMsg;

    [Header("Pose Sensor Settings")]
    public GameObject target;

    public override void Initialize() {
        pose2DMsg = new Pose2DMsg();
    }
    
    public override void GetSensorData() {

        // Convert Unity data to ROS message
        pose2DMsg.x = target.transform.position.x;
        pose2DMsg.y = target.transform.position.z;
        pose2DMsg.theta = -target.transform.eulerAngles.y;

    }

    protected override void UpdateSensor() {
    }

    public override void ResetSensor() {
    }
}
