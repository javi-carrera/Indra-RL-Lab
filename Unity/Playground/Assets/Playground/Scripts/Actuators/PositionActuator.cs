using System.Collections;
using System.Collections.Generic;
using RosMessageTypes.Geometry;
using UnityEngine;

public class PositionActuator : Actuator<Vector3Msg> {

    public GameObject target;
    public float speed;
    private Vector3 _targetPosition;

    public override void SetData(Vector3Msg msg) {

        // Convert ROS message to Unity data
        _targetPosition = new Vector3(
            (float)msg.x, 
            (float)msg.y, 
            (float)msg.z
        );
    }

    void Update() {
        target.transform.position = Vector3.MoveTowards(transform.position, _targetPosition, speed * Time.deltaTime);
    }



}
