using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;



public class Damageable : MonoBehaviour {

    public float maxHealth;
    [HideInInspector] public float health;

    public event Action OnDeath;


    private void Start() {
        health = maxHealth;
    }

    public void TakeDamage(float damage) {

        // Decrease the health
        health = (health - damage) < 0 ? 0 : health - damage;

        // Check if the object is dead
        if (health == 0) Die();

    }

    private void Die() {
        OnDeath?.Invoke();
    }
}
