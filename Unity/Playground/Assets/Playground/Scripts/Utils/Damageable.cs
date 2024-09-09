using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;


[RequireComponent(typeof(Collider))]
public class Damageable : MonoBehaviour {

    public float maxHealth;
    public float damageOnCollision;
    public float maxCollisionDamageCooldown;
    [HideInInspector] public float health;

    public event Action OnDeath;

    private float _collisionDamageCooldown;
    private bool _isColliding;



    private void Start() {
        health = maxHealth;
        _collisionDamageCooldown = maxCollisionDamageCooldown;
    }

    private void Update() {

        // Decrease the collision damage cooldown
        _collisionDamageCooldown = (_collisionDamageCooldown - Time.deltaTime) > 0 ? _collisionDamageCooldown - Time.deltaTime : 0;


        if (_isColliding && _collisionDamageCooldown == 0) {
            TakeDamage(damageOnCollision);
            _collisionDamageCooldown = maxCollisionDamageCooldown;
        }
        

    }

    public void TakeDamage(float damage) {

        // Decrease the health
        health = (health - damage) < 0 ? 0 : health - damage;

        // Check if the object is dead
        if (health == 0) Die();

    }

    private void OnCollisionEnter(Collision collision) {

        if (collision.gameObject.layer != LayerMask.NameToLayer("Ground")) {
            _isColliding = true;
        }

    }

    private void OnCollisionExit(Collision collision) {

        if (collision.gameObject.layer != LayerMask.NameToLayer("Ground")) {
            _isColliding = false;
        }

    }

    

    private void Die() {
        OnDeath?.Invoke();
    }
}
