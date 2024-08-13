using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public interface ISensor {

    public string SensorName { get; }

    void GetData();
    void ResetSensor();
}

public abstract class Sensor: MonoBehaviour, ISensor {
    
    public string sensorName;

    /// <summary>
    /// Convert Unity data to ROS message
    /// </summary>
    public abstract void GetData();

    /// <summary>
    /// [TODO]
    /// </summary>
    public abstract void ResetSensor();


    // Implement ISensor
    string ISensor.SensorName => sensorName;
    void ISensor.GetData() => GetData();
    void ISensor.ResetSensor() => ResetSensor();
}
