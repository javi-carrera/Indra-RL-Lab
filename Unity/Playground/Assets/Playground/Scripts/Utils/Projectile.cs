using System;
using System.Collections;
using System.Collections.Generic;
using UnityEditor.Callbacks;
using UnityEngine;


[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(Collider))]
public class Projectile : MonoBehaviour {

    public float damage;
    public float damageRadius;
    public float damagePercentageAtMaxRadius;
    public float explosionForce;

    private bool _hasHit = false;

    public event Action<GameObject> OnCollisionEnterEvent;


    private void OnEnable() {
        _hasHit = false;
    }
    
    private void OnCollisionEnter(Collision collision) {

        OnCollisionEnterEvent?.Invoke(gameObject);
            
        if (!_hasHit) {
            _hasHit = true;
            Explode();
        }
    }

    private void Explode() {

        Collider[] colliders;

        // Get all colliders in the damage radius
        colliders = Physics.OverlapSphere(transform.position, damageRadius);

        foreach (Collider collider in colliders) {

            // Apply damage to the damageable objects
            if (collider.TryGetComponent<Damageable>(out var damageableObject)) {

                // Calculate the adjusted damage
                float distance = Vector3.Distance(transform.position, damageableObject.transform.position);
                float adjustedDamage = damage * Mathf.Exp(Mathf.Log(damagePercentageAtMaxRadius * distance / damageRadius));

                // Apply the damage to the damageable object
                damageableObject.TakeDamage(adjustedDamage);
            }
        }

        // Get all colliders in the damage radius
        colliders = Physics.OverlapSphere(transform.position, damageRadius);

        foreach (Collider collider in colliders) {

            // Apply explosion force to the rigidbodies
            if (collider.TryGetComponent<Rigidbody>(out var rb)) {
                rb.AddExplosionForce(explosionForce, transform.position, damageRadius);
            }
        }
    }
}
