using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;


[RequireComponent(typeof(Collider))]
public class Damageable : MonoBehaviour {

    public float maxHealth;
    public float damageOnCollision;
    public float collisionDamageCooldown;
    [HideInInspector] public float health;

    public event Action OnDeath;

    private bool _isColliding;



    private void Start() {
        health = maxHealth;
    }

    public void TakeDamage(float damage) {

        // Decrease the health
        health = (health - damage) < 0 ? 0 : health - damage;

        // Check if the object is dead
        if (health == 0) Die();

    }

    private void OnCollisionEnter(Collision collision) {

        // Check if the object is not colliding with the ground
        if (collision.gameObject.layer != LayerMask.NameToLayer("Ground")) {
            _isColliding = true;
        }
    }

    private void Die() {
        OnDeath?.Invoke();
    }
}
