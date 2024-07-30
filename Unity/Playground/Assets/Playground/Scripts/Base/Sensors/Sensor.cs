using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public abstract class Sensor: MonoBehaviour {
    
    /// <summary>
    /// Convert Unity data to ROS message
    /// </summary>
    public abstract void GetData();

    public abstract void ResetSensor();
}
