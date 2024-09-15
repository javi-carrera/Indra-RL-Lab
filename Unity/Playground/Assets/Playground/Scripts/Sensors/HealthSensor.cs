using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.InterfacesPkg;

public class HealthSensor : Sensor {
    
    [HideInInspector] public HealthInfoMsg healthInfoMsg;

    [Header("Health Sensor Settings")]
    public Damageable damageable;

    public override void Initialize() {
        healthInfoMsg = new HealthInfoMsg();
    }

    public override void GetSensorData() {

        // Convert Unity data to ROS message
        healthInfoMsg.health = damageable.health;
        healthInfoMsg.max_health = damageable.maxHealth;

    }

    protected override void UpdateSensor() {
    }

    public override void ResetSensor() {

        
        damageable.health = damageable.maxHealth;

        
    }
}
