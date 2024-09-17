using System.Collections;
using System.Collections.Generic;
using UnityEngine;



[RequireComponent(typeof(Damageable))]
public class Destructible : MonoBehaviour {
    
    public GameObject destroyedObjectPrefab;
    public ParticleSystem destructionParticles;

    private void Start() {
        GetComponent<Damageable>().OnDeath += DestroyObject;
    }

    private void DestroyObject() {

        // Get the position and rotation of the object
        Vector3 position = transform.position;
        Quaternion rotation = transform.rotation;

        // Play the destruction particles
        PlayDestructionParticles();

        // Set the object inactive
        gameObject.SetActive(false);

        // Instantiate the destroyed object prefab
        GameObject destroyedObject = Instantiate(destroyedObjectPrefab, position, rotation);

    }

    private void PlayDestructionParticles() {

        // Instantiate the destruction particles
        ParticleSystem particles = Instantiate(destructionParticles, transform.position, transform.rotation);

        // Play the particles
        particles.Play();

        // Destroy the particles after the duration
        Destroy(particles.gameObject, particles.main.duration);
    }

}
