using System;
using System.Collections;
using System.Collections.Generic;
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


    private void Start() {

        // Initialize the exploded flag
        _hasExploded = false;

        // Explode after the max lifetime
        _ = ExplodeAfterTime();

    }
    
    private void OnTriggerEnter(Collider other) {
        
        // Explode when hitting a collider
        if (!_hasExploded) Explode(other);
    }

    private void Explode(Collider other = null) {

        // Set the exploded flag
        _hasExploded = true;

        // Play the explosion effect
        PlayExplosionEffect();

        // Get all colliders in the damage radius
        foreach (Collider collider in Physics.OverlapSphere(transform.position, damageRadius)) {

            // Apply damage to the damageable objects
            if (collider.TryGetComponent<Damageable>(out var damageableObject)) {

                // Calculate the adjusted damage
                float adjustedDamage = collider == other ? damage : CalculateDamage(Vector3.Distance(transform.position, damageableObject.transform.position));

                // Apply the damage to the damageable object
                damageableObject.TakeDamage(adjustedDamage);
            }

            // Apply explosion force to the rigidbodies
            if (collider.TryGetComponent<Rigidbody>(out var rb)) {
                rb.AddExplosionForce(explosionForce, transform.position, damageRadius);
            }
        }

        // Destroy the projectile
        Destroy(gameObject);

    }


    private float CalculateDamage(float distance) {

        // Calculate the adjusted damage
        return damage * Mathf.Exp(Mathf.Log(damagePercentageAtMaxRadius) * distance / damageRadius);
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

        // Explode after the max lifetime
        await Task.Delay((int)(maxLifetime * 1000));
        if (!_hasExploded) Explode();
    }
}
