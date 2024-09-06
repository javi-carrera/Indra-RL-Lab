using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(Collider))]
public class Enemy : MonoBehaviour {

    public float maxHealth;
    private float _health;


    private void Start() {
        _health = maxHealth;
    }

    private void OnCollisionEnter(Collision collision) {
        
        if (collision.gameObject.TryGetComponent<Projectile>(out var projectile)) {
            TakeDamage(projectile.damage);
        }
    }

    public void TakeDamage(float damage) {

        _health = Mathf.Max(0, _health - damage);

        if (_health == 0)
            Die();

    }


    private void Die() {
        
        if (TryGetComponent<Destructible>(out var destructibleObject)) {
            destructibleObject.DestroyObject();
        }
        else {
            Destroy(gameObject);
        }
    }





}
