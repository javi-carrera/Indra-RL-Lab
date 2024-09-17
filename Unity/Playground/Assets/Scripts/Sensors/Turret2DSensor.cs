using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.InterfacesPkg;

public class Turret2DSensor : Sensor {
    
    [HideInInspector] public Turret2DSensorMsg turret2DSensorMsg;


    [Header("Turret 2D Sensor Settings")]
    public Turret2DActuator turret2DActuator;


    public override void Initialize() {
        turret2DSensorMsg = new Turret2DSensorMsg();
    }

    public override void GetSensorData() {
        // Convert Unity data to ROS message
        turret2DSensorMsg.current_angle = turret2DActuator.currentAngle;
        turret2DSensorMsg.fire_rate = turret2DActuator.fireRate;
        turret2DSensorMsg.cooldown = turret2DActuator.cooldown;
        turret2DSensorMsg.has_fired = turret2DActuator.hasFired;
    }

    protected override void UpdateSensor() {
    }

    public override void ResetSensor() {
    }


}
