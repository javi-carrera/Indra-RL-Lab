using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Geometry;

public class PoseSensor : Sensor {

    [HideInInspector] public PoseMsg poseMsg;

    [Header("Pose Sensor Settings")]
    public GameObject target;

    public override void Initialize() {
        poseMsg = new PoseMsg();
    }
    
    public override void GetSensorData() {

        // Convert Unity data to ROS message
        poseMsg.position = PoseConverter.Unity2RosPosition(target.transform.position);
        poseMsg.orientation = PoseConverter.Unity2RosRotation(target.transform.rotation);

    }

    protected override void UpdateSensor() {
    }

    public override void ResetSensor() {
    }
}
