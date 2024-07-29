using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

public abstract class Actuator<T> : MonoBehaviour where T : Message, new() {

    // TODO: test with 'as' keyword method

    /// <summary>
    /// Convert ROS message to Unity data
    /// </summary>
    public abstract void SetData(T msg);
}
