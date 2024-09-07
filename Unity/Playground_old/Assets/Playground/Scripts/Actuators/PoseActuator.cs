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
    [HideInInspector] public Quaternion targetRotation;
    [HideInInspector] public Vector3 targetPosition;
    

    public override void Initialize() {
    }

    public override void SetActuatorData(PoseMsg msg) {

        // Convert ROS pose message to Unity data

        targetPosition = PoseConverter.Ros2UnityPosition(msg.position);
        targetRotation = PoseConverter.Ros2UnityRotation(msg.orientation);

        if (teleport) {

            // Teleport to target position
            target.transform.SetPositionAndRotation(targetPosition, targetRotation);
            return;
        }
    }

    protected override void UpdateActuator() {
        
        // If teleport is enabled, return
        if (teleport) return;

        // Move towards target position and rotation
        target.transform.SetPositionAndRotation(
            Vector3.MoveTowards(target.transform.position, targetPosition, positionSpeed * Time.deltaTime), 
            Quaternion.RotateTowards(target.transform.rotation, targetRotation, rotationSpeed * Time.deltaTime)
        );
    }

    public override void ResetActuator() {
    }
}
