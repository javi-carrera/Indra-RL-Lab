using System.Collections;
using System.Collections.Generic;
using RosMessageTypes.InterfacesPkg;
using UnityEngine;

public class Pose2DActuator : Actuator<Pose2DMsg> {

    [Header("Pose Actuator Settings")]
    public GameObject target;
    public bool teleport;
    public float positionSpeed;
    public float rotationSpeed;
    [HideInInspector] public Quaternion targetRotation;
    [HideInInspector] public Vector3 targetPosition;
    

    public override void Initialize() {
    }

    public override void SetActuatorData(Pose2DMsg msg) {

        // Convert ROS pose message to Unity data

        targetPosition = new Vector3(msg.x, target.transform.position.y, msg.y);
        targetRotation = Quaternion.Euler(target.transform.rotation.eulerAngles.x, -msg.theta, target.transform.rotation.eulerAngles.z);

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
