using System.Collections;
using System.Collections.Generic;
using RosMessageTypes.InterfacesPkg;
using UnityEditor.Experimental.GraphView;
using UnityEngine;


public class TriggerSensor : Sensor {

    
    [HideInInspector] public TriggerSensorMsg triggerSensorMsg;
    private TriggerSensorHandler _triggerSensorHandler;

    [Header("Trigger Sensor Settings")]
    public GameObject target;
    public float maxTimerCount;
    private float _timerCount;
    private bool _hasTriggered;
    private bool _hasTimerFinished;

    [Header("Debug Settings")]
    public Color hasTriggeredColor;
    private Color _originalColor;
    private Renderer _renderer;


    void OnEnable() {

        // Check if target has TriggerSensorHandler component
        if (target.GetComponent<TriggerSensorHandler>() == null) {
            _triggerSensorHandler = target.AddComponent<TriggerSensorHandler>();
        }

        _triggerSensorHandler.OnTriggerEnterEvent += HandleTriggerEnter;
        _triggerSensorHandler.OnTriggerExitEvent += HandleTriggerExit;
    }

    void OnDisable() {
        _triggerSensorHandler.OnTriggerEnterEvent -= HandleTriggerEnter;
        _triggerSensorHandler.OnTriggerExitEvent -= HandleTriggerExit;
    }

    void Start() {

        // Reset sensor
        ResetSensor();

        // Initialize sensor message
        _renderer = target.GetComponent<Renderer>();
        _originalColor = _renderer.material.color;

    }

    void Update() {

        if (_hasTriggered) {

            // Update timer until it reaches maxTimerCount
            _timerCount = (_timerCount + Time.deltaTime < maxTimerCount) ? _timerCount + Time.deltaTime : maxTimerCount;

            // Raise flag when timer reaches maxTimerCount
            _hasTimerFinished = _timerCount >= maxTimerCount;
        }

        // Lerp the material between the original and the hasTriggeredMaterial
        if (maxTimerCount > 0) {
            _renderer.material.color = _hasTriggered ? Color.Lerp(_originalColor, hasTriggeredColor, _timerCount / maxTimerCount) : _originalColor;
        }
        else {
            _renderer.material.color = _hasTriggered ? hasTriggeredColor : _originalColor;
        }   
    }


    void HandleTriggerEnter(Collider other) {
        // Set trigger
        _hasTriggered = true;
    }

    void HandleTriggerExit(Collider other) {
    
        // Reset trigger
        _timerCount = 0.0f;
        _hasTriggered = false;
        _hasTimerFinished = false;
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
        _timerCount = 0.0f;
        _hasTriggered = false;
        _hasTimerFinished = false;
        
    }
}
