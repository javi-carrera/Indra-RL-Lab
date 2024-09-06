using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;


public interface IActuator {

    void Initialize();
    void SetActuatorData(object msg);
    void ResetActuator();
}

public abstract class Actuator<T> : MonoBehaviour, IActuator
    where T : Message, new() {


    void Update() {
        UpdateActuator();
    }


    /// <summary>
    /// [TODO]
    /// </summary>
    public abstract void Initialize();

    /// <summary>
    /// Convert ROS message to Unity data
    /// </summary>
    public abstract void SetActuatorData(T msg);

    /// <summary>
    /// [TODO]
    /// </summary>
    protected abstract void UpdateActuator();

    /// <summary>
    /// [TODO]
    /// </summary>
    public abstract void ResetActuator();


    // Implement IActuator
    void IActuator.Initialize() => Initialize();
    void IActuator.SetActuatorData(object msg) => SetActuatorData((T)msg);
    void IActuator.ResetActuator() => ResetActuator();
}
