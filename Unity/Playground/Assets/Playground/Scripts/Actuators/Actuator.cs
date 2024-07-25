using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

public abstract class Actuator<T> : MonoBehaviour where T : Message, new() {
    public abstract void SetData(T msg);
}
