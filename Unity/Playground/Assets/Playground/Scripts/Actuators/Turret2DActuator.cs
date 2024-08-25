using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Threading.Tasks;

public class Turret2DActuator : MonoBehaviour {
    
    [Header("Turret 2D Actuator Settings")]
    public Transform target;
    public Transform shootingPoint;
    public GameObject bulletPrefab;
    public float rotationSpeed;
    public float targetAngle;
    public float shootVelocity;
    public float cooldown; // Bullets per second
    public int poolSize;
    public float maxBulletLifetime;

    private ObjectPooler _objectPooler;
    private float nextTimeToFire = 0f;

    private void Start() {
        _objectPooler = new ObjectPooler(bulletPrefab, poolSize);
    }

    private void Update() {

        UpdateActuator();

        if (Input.GetButton("Fire1") && Time.time >= nextTimeToFire) {
            nextTimeToFire = Time.time + 1f / cooldown;
            Shoot();
        }
    }

    private void UpdateActuator() {

        Quaternion targetRotation = Quaternion.Euler(0, targetAngle, 0);

        // Convert target rotation to global coordinates
        targetRotation = transform.rotation * targetRotation;

        target.transform.rotation = Quaternion.RotateTowards(target.transform.rotation, targetRotation, Time.deltaTime * rotationSpeed);
    }

    void Shoot() {

        GameObject bullet = _objectPooler.GetPooledObject();
        
        if (bullet != null) {

            Rigidbody rb = bullet.GetComponent<Rigidbody>();

            bullet.transform.SetPositionAndRotation(shootingPoint.position, shootingPoint.rotation);

            rb.AddForce(shootingPoint.right * shootVelocity, ForceMode.VelocityChange);

            _ = DeactivateBulletAfterLifetime(bullet);

        }
    }


    private async Task DeactivateBulletAfterLifetime(GameObject bullet) {
        await Task.Delay((int)(maxBulletLifetime * 1000));
        _objectPooler.ReturnToPool(bullet);
    }
}
