using System.Collections;
using System.Collections.Generic;
using RosMessageTypes.Geometry;
using UnityEngine;

public class PoseActuator : Actuator<PoseMsg> {

    [Header("Pose Actuator Settings")]
    public GameObject target;
    public bool teleport;
    public float positionSpeed;
    public float rotationSpeed;
    private Quaternion _targetRotation;
    private Vector3 _targetPosition;
    

    public override void Initialize() {
    }

    public override void SetActuatorData(PoseMsg msg) {

        // Convert ROS pose message to Unity data

        _targetPosition =PoseConverter.Ros2UnityPosition(msg.position);
        _targetRotation = PoseConverter.Ros2UnityRotation(msg.orientation);

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
