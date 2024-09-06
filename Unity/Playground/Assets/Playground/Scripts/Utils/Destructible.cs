using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Threading.Tasks;

[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(Collider))]
public class Destructible : MonoBehaviour {
    

    public GameObject destroyedObjectPrefab; 

    public void DestroyObject() {

        // Get the position and rotation of the object
        transform.GetPositionAndRotation(out Vector3 position, out Quaternion rotation);

        // Get the velocity and angular velocity of the object
        Vector3 velocity = GetComponent<Rigidbody>().velocity;
        Vector3 angularVelocity = GetComponent<Rigidbody>().angularVelocity;

        // Destroy the object
        Destroy(gameObject);

        // Instantiate the destroyed object prefab
        GameObject destroyedObject = Instantiate(destroyedObjectPrefab, position, rotation);

        // Get the rigidbodies of the child objects in the destroyed object
        Rigidbody[] destroyedObjectRigidbodies = destroyedObject.GetComponentsInChildren<Rigidbody>();

        // Set the velocity and angular velocity of the destroyed object rigidbodies
        foreach (Rigidbody rb in destroyedObjectRigidbodies) {
            rb.velocity = velocity;
            rb.angularVelocity = angularVelocity;

        }

        // Deactivate the destroyed object after a certain time
        _ = DeactivateGameObjectAfterTime(destroyedObject, 5.0f);


    }


    private async Task DeactivateGameObjectAfterTime(GameObject gameObject, float time) {
        await Task.Delay((int)(time * 1000));
        Destroy(gameObject);
    }
}
