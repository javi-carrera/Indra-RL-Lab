using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ObjectPooler {
    public GameObject objectPrefab;
    public int poolSize;
    private Queue<GameObject> objectPool = new Queue<GameObject>();

    public ObjectPooler(GameObject prefab, int size) {

        objectPrefab = prefab;
        poolSize = size;

        for (int i = 0; i < poolSize; i++) {
            GameObject obj = GameObject.Instantiate(objectPrefab);
            obj.SetActive(false);
            objectPool.Enqueue(obj);
        }
    }

    ~ObjectPooler() {
        foreach (GameObject obj in objectPool) {
            GameObject.Destroy(obj);
        }
        objectPool.Clear();
    }

    public GameObject GetPooledObject() {

        if (objectPool.Count > 0) {
            GameObject obj = objectPool.Dequeue();
            obj.SetActive(true);
            return obj;
        }
        return null; // Optionally expand the pool here if needed
    }

    public void ReturnToPool(GameObject obj) {

        // Try to access the object's rigidbody and reset its velocity
        if (obj.TryGetComponent<Rigidbody>(out var rb)) {
            rb.velocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }

        obj.SetActive(false);
        objectPool.Enqueue(obj);
    }
    
}

