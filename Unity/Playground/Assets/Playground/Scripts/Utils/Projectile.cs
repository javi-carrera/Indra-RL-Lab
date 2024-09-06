using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Projectile : MonoBehaviour {

    public float damage;
    public event Action<GameObject> OnCollisionEnterEvent;

    private bool hasHit = false;

    private void OnCollisionEnter(Collision collision) {
        OnCollisionEnterEvent?.Invoke(gameObject);
    }
}
