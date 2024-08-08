using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;


public interface IActuator {
    void SetData(object msg);
    void ResetActuator();
}

public abstract class Actuator<T> : MonoBehaviour, IActuator
    where T : Message, new() {

    public string actuatorName;

    /// <summary>
    /// Convert ROS message to Unity data
    /// </summary>
    public abstract void SetData(T msg);

    /// <summary>
    /// [TODO]
    /// </summary>
    public abstract void ResetActuator();


    // Implement IActuator
    void IActuator.SetData(object msg) => SetData((T)msg);
    void IActuator.ResetActuator() => ResetActuator();
}
