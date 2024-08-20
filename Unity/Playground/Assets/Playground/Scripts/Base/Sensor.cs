using System.Collections;
using System.Collections.Generic;
using System.Xml.Serialization;
using UnityEngine;


public interface ISensor {

    public string SensorName { get; }

    void Initialize();
    void GetSensorData();
    void ResetSensor();

}

public abstract class Sensor: MonoBehaviour, ISensor {
    
    public string sensorName;


    void Update() {
        UpdateSensor();
    }

    /// <summary>
    /// [TODO]
    /// </summary>
    public abstract void Initialize();

    /// <summary>
    /// Convert Unity data to ROS message
    /// </summary>
    public abstract void GetSensorData();


    /// <summary>
    /// [TODO]
    /// </summary>
    protected abstract void UpdateSensor();


    /// <summary>
    /// [TODO]
    /// </summary>
    public abstract void ResetSensor();



    // Implement ISensor
    string ISensor.SensorName => sensorName;
    void ISensor.Initialize() => Initialize();
    void ISensor.GetSensorData() => GetSensorData();
    void ISensor.ResetSensor() => ResetSensor();
}
