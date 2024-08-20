using System.Collections;
using System.Collections.Generic;
using RosMessageTypes.Geometry;
using UnityEngine;

public class PoseActuator : Actuator<PoseMsg> {

    [Header("Pose Actuator Settings")]
    public GameObject target;
    [HideInInspector] public Vector3 environmentPosition;
    public bool teleport;
    public float positionSpeed;
    public float rotationSpeed;
    private Quaternion _targetRotation;
    private Vector3 _targetPosition;
    

    public override void Initialize() {
    }

    public override void SetActuatorData(PoseMsg msg) {

        // Convert ROS pose message to Unity data

        _targetPosition = new Vector3(
            (float)msg.position.x + environmentPosition.x,
            (float)msg.position.y + environmentPosition.y,
            (float)msg.position.z + environmentPosition.z
        );

        _targetRotation = new Quaternion(
            (float)msg.orientation.x,
            (float)msg.orientation.y,
            (float)msg.orientation.z,
            (float)msg.orientation.w
        );

        if (teleport) {

            // Teleport to target position
            target.transform.SetPositionAndRotation(_targetPosition, _targetRotation);
            return;
        }
    }

    protected override void UpdateActuator() {

        if (teleport) {
            return;
        }

        // Move towards target position
        
        // Rotate towards target rotation
        target.transform.SetPositionAndRotation(
            Vector3.MoveTowards(target.transform.position, _targetPosition, positionSpeed * Time.deltaTime), 
            Quaternion.RotateTowards(target.transform.rotation, _targetRotation, rotationSpeed * Time.deltaTime)
        );
    }

    public override void ResetActuator() {
    }
}
