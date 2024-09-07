// Project: Playground
// File: LidarSensor.cs
// Authors: Guillermo Escolano
// License: Apache 2.0 (refer to LICENSE file in the project root)

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class HealthBar : MonoBehaviour {
    
    
    public Slider currentHealthBarSlider;
    public Slider easeHealthBarSlider;
    public float maxHealth = 100;
    public float currentHealth = 100;

    public float lerpSpeed = 0.02f;

    private Transform _camera;


    private void Start() {

        // Get the camera
        _camera = Camera.main.transform;

        // Set the max health
        currentHealth = maxHealth;
    }

    private void Update() {

        if(Input.GetKeyDown(KeyCode.Space))
            TakeDamage(20);

        // Update the health bar
        if(currentHealthBarSlider.value != currentHealth)
            currentHealthBarSlider.value = currentHealth;

        // Update the ease health bar
        if(currentHealthBarSlider.value != easeHealthBarSlider.value)
            easeHealthBarSlider.value = Mathf.Lerp(easeHealthBarSlider.value, currentHealthBarSlider.value, lerpSpeed);
    }

    

    private void LateUpdate() {
        transform.LookAt(transform.position + _camera.forward);
    }

    private void TakeDamage(float damage) {

        // Decrease the health
        currentHealth = (currentHealth - damage) < 0 ? 0 : currentHealth - damage;

    }


}
