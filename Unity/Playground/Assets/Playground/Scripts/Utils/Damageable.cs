using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(Collider))]
public class Damageable : MonoBehaviour {

    public float maxHealth;
    private float _health;

    public event Action OnDeath;


    private void Start() {
        _health = maxHealth;
    }

    public void TakeDamage(float damage) {

        _health = Mathf.Max(0, _health - damage);

        if (_health == 0)
            Die();

    }


    private void Die() {
        OnDeath?.Invoke();
    }





}
