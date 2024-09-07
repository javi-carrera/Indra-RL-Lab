using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Threading.Tasks;

[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(Collider))]
[RequireComponent(typeof(Damageable))]
public class Destructible : MonoBehaviour {

    public GameObject destroyedObjectPrefab;
    public float destroyTime;

    private void Start() {
        GetComponent<Damageable>().OnDeath += DestroyObject;
    }

    private void DestroyObject() {

        // Get the position and rotation of the object
        Vector3 position = transform.position;
        Quaternion rotation = transform.rotation;

        // Destroy the object
        Destroy(gameObject);

        // Instantiate the destroyed object prefab
        GameObject destroyedObject = Instantiate(destroyedObjectPrefab, position, rotation);

        // Deactivate the destroyed object after a certain time
        _ = DestroyGameObjectAfterTime(destroyedObject, destroyTime);


    }

    private async Task DestroyGameObjectAfterTime(GameObject other, float time) {
        await Task.Delay((int)(time * 1000));
        if (other != null) Destroy(other);
    }
}
