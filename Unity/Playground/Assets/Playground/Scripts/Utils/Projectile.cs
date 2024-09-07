using System;
using System.Collections;
using System.Collections.Generic;
using UnityEditor.Callbacks;
using UnityEngine;
using System.Threading.Tasks;


[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(Collider))]
public class Projectile : MonoBehaviour {

    public float damage;
    public float damageRadius;
    public float damagePercentageAtMaxRadius;
    public float explosionForce;
    public float maxLifetime;
    public ParticleSystem explosionParticles;

    private bool _hasExploded;

    // public event Action<GameObject> OnExplodeEvent;


    private void Start() {

        _hasExploded = false;

        // Explode after the max lifetime
        _ = ExplodeAfterTime();

    }
    
    private void OnTriggerEnter(Collider other) {
            
        if (!_hasExploded) Explode();
    }

    private void Explode() {

        // Set the exploded flag
        _hasExploded = true;

        

        // Play the explosion effect
        PlayExplosionEffect();


        Collider[] colliders;

        // Get all colliders in the damage radius
        colliders = Physics.OverlapSphere(transform.position, damageRadius);

        foreach (Collider collider in colliders) {

            // Apply damage to the damageable objects
            if (collider.TryGetComponent<Damageable>(out var damageableObject)) {

                // Calculate the adjusted damage
                float distance = Vector3.Distance(transform.position, damageableObject.transform.position);
                // float adjustedDamage = damage * Mathf.Exp(Mathf.Log(damagePercentageAtMaxRadius * distance / damageRadius));
                float adjustedDamage = damage;

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


        // Invoke the explode event
        // OnExplodeEvent?.Invoke(gameObject);

        // Destroy the projectile
        Destroy(gameObject);

    }


    private void PlayExplosionEffect() {

        // Unparent the particles from the shell.
        explosionParticles.transform.parent = null;

        // Play the particle system.
        explosionParticles.Play();

        // Once the particles have finished, destroy the gameobject they are on.
        ParticleSystem.MainModule mainModule = explosionParticles.main;
        Destroy (explosionParticles.gameObject, mainModule.duration);

    }

    private async Task ExplodeAfterTime() {

        await Task.Delay((int)(maxLifetime * 1000));

        if (!_hasExploded) Explode();
    }
}
