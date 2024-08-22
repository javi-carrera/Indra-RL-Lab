using System;
using System.Collections;
using System.Collections.Generic;
using RosMessageTypes.InterfacesPkg;
using UnityEngine;


public class TriggerSensorHandler : MonoBehaviour {

    public event Action<Collider> OnTriggerEnterEvent;
    public event Action<Collider> OnTriggerExitEvent;


    void OnEnable() {
        
        // Add a collider if it doesn't exist
        if (GetComponent<Collider>() == null) {
            gameObject.AddComponent<BoxCollider>();
        }

        // Set the collider as trigger
        GetComponent<Collider>().isTrigger = true;

    }

    void OnTriggerEnter(Collider other) {
        OnTriggerEnterEvent?.Invoke(other);
    }

    void OnTriggerExit(Collider other) {
        OnTriggerExitEvent?.Invoke(other);
    }

}
