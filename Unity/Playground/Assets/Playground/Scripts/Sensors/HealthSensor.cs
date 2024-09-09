using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.InterfacesPkg;

public class HealthSensor : Sensor {
    
    [HideInInspector] public HealthSensorMsg healthSensorMsg;

    [Header("Health Sensor Settings")]
    public Damageable damageable;

    public override void Initialize() {
        healthSensorMsg = new HealthSensorMsg();
    }

    public override void GetSensorData() {

        // Convert Unity data to ROS message
        healthSensorMsg.health = damageable.health;
        healthSensorMsg.max_health = damageable.maxHealth;

    }

    protected override void UpdateSensor() {
    }

    public override void ResetSensor() {

        
        damageable.health = damageable.maxHealth;

        
    }
}
