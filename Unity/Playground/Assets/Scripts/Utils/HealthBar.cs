// Project: Playground
// File: LidarSensor.cs
// Authors: Guillermo Escolano
// License: Apache 2.0 (refer to LICENSE file in the project root)

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class HealthBar : MonoBehaviour {
    
    public Damageable damageable;
    public float lerpSpeed = 0.02f;
    public Slider currentHealthBarSlider;
    public Slider easeHealthBarSlider;


    private float _maxHealth;
    private float _currentHealth;
    private Transform _camera;


    private void Start() {

        // Get the camera
        _camera = Camera.main.transform;

        // Set the max health
        _maxHealth = damageable.maxHealth;
        _currentHealth = _maxHealth;

        // Add the event listeners
        // damageable.OnTakeDamage += TakeDamage;
    }

    private void Update() {
        
        // Update the health
        _currentHealth = damageable.health;

        // Update the health bar
        if(currentHealthBarSlider.value != _currentHealth)
            currentHealthBarSlider.value = _currentHealth;

        // Update the ease health bar
        if(currentHealthBarSlider.value != easeHealthBarSlider.value)
            easeHealthBarSlider.value = Mathf.Lerp(easeHealthBarSlider.value, currentHealthBarSlider.value, lerpSpeed);

        // Look at the camera
        transform.LookAt(transform.position + _camera.forward);
    }

    

    private void LateUpdate() {
        
    }


}
