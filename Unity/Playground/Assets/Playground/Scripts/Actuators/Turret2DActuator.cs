using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Threading.Tasks;
using RosMessageTypes.InterfacesPkg;

public class Turret2DActuator : Actuator<TurretActuatorMsg> {
    
    [Header("Turret 2D Actuator Settings")]
    public Transform target;
    public Transform shootingPoint;
    public GameObject bulletPrefab;
    public float rotationSpeed;
    public float targetAngle;
    public float shootVelocity;
    public float fireRate; // Bullets per second
    public int poolSize;
    public float maxBulletLifetime;

    private ObjectPooler _objectPooler;
    private float _nextTimeToFire = 0f;


    public override void Initialize() {
        ResetActuator();
    }



    public override void SetActuatorData(TurretActuatorMsg msg) {
        targetAngle = msg.target_angle;
    }

    public override void ResetActuator() {
        targetAngle = 0;
    }





    private void Start() {
        _objectPooler = new ObjectPooler(bulletPrefab, poolSize);
    }

    private void Update() {

        UpdateActuator();

        if (Input.GetButton("Fire1") && Time.time >= _nextTimeToFire) {
            _nextTimeToFire = Time.time + 1.0f / fireRate;
            Shoot();
        }
    }

    protected override void UpdateActuator() {

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
