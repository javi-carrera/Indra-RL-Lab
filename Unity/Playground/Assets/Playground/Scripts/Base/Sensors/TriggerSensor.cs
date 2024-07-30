using System.Collections;
using System.Collections.Generic;
using RosMessageTypes.InterfacesPkg;
using UnityEditor.Experimental.GraphView;
using UnityEngine;

[RequireComponent(typeof(Collider))]
public class TriggerSensor : Sensor {

    private bool _hasTimerFinished;
    private bool _hasTriggered;
    private float _timerCount;
    private Color _originalColor;
    
    [HideInInspector]
    public TriggerSensorMsg triggerSensorMsg;
    public float maxTimerCount;
    public Color hasTriggeredColor;

    void Start() {

        // Reset sensor
        ResetSensor();

        // Initialize sensor message
        _originalColor = GetComponent<Renderer>().material.color;

    }

    void Update() {

        if (_hasTriggered) {

            // Update timer until it reaches maxTimerCount
            _timerCount = (_timerCount + Time.deltaTime < maxTimerCount) ? _timerCount + Time.deltaTime : maxTimerCount;

            // Raise flag when timer reaches maxTimerCount
            _hasTimerFinished = _timerCount >= maxTimerCount;
        }

        // Lerp the material between the original and the hasTriggeredMaterial
        if (maxTimerCount > 0){
            GetComponent<Renderer>().material.color = _hasTriggered ? Color.Lerp(_originalColor, hasTriggeredColor, _timerCount / maxTimerCount) : _originalColor;
        }
        else {
            GetComponent<Renderer>().material.color = _hasTriggered ? hasTriggeredColor : _originalColor;
        }   
    }

    void OnTriggerEnter(Collider other) {
        // Set trigger
        _hasTriggered = true;
    }

    void OnTriggerExit(Collider other) {
    
        // Reset trigger
        _hasTriggered = false;
        _hasTimerFinished = false;
        _timerCount = 0.0f;
    
    }

    public override void GetData() {

        // Convert Unity data to ROS message
        triggerSensorMsg.has_triggered = _hasTriggered;
        triggerSensorMsg.has_timer_finished = _hasTimerFinished;
        triggerSensorMsg.timer_count = _timerCount;
        triggerSensorMsg.max_timer_count = maxTimerCount;

    }

    public override void ResetSensor() {

        // Reset sensor
        _hasTriggered = false;
        _hasTimerFinished = false;
        _timerCount = 0.0f;
    }
}
