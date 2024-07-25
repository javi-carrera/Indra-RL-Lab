using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Geometry;

public class PositionSensor : Sensor {

    [HideInInspector]
    public Vector3Msg position;

    public override void GetData() {

        // Convert Unity data to ROS message
        position.x = transform.position.x;
        position.y = transform.position.y;
        position.z = transform.position.z;

    }
}
