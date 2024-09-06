using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Threading.Tasks;
using RosMessageTypes.InterfacesPkg;
using Unity.VisualScripting;

public class Turret2DActuator : Actuator<Turret2DActuatorMsg> {
    
    [Header("Turret 2D Actuator Settings")]
    public Transform target;
    public Transform shootingPoint;
    public GameObject bulletPrefab;
    public float rotationSpeed;
    public float shootVelocity;
    public float fireRate;
    public float maxBulletLifetime;

    [HideInInspector] public float targetAngle;
    [HideInInspector] public bool fire;

    private ObjectPooler _objectPooler;
    private int _poolSize;
    private float _nextTimeToFire = 0f;
    private Vector3 _previousShootingPointPosition;
    private Vector3 _shootingPointVelocity;


    public override void Initialize() {

        _poolSize = (int)(maxBulletLifetime * fireRate);
        _objectPooler = new ObjectPooler(bulletPrefab, _poolSize);

        ResetActuator();

    }



    public override void SetActuatorData(Turret2DActuatorMsg msg) {

        targetAngle = msg.target_angle;
        fire = msg.fire;
    }

    public override void ResetActuator() {

        targetAngle = 0.0f;
        fire = false;

        _previousShootingPointPosition = shootingPoint.position;
        _shootingPointVelocity = Vector3.zero;
    }

    protected override void UpdateActuator() {

        Quaternion targetRotation = Quaternion.Euler(0, targetAngle, 0);

        // Convert target rotation to global coordinates
        targetRotation = transform.rotation * targetRotation;

        target.transform.rotation = Quaternion.RotateTowards(target.transform.rotation, targetRotation, Time.deltaTime * rotationSpeed);


        if (fire && Time.time >= _nextTimeToFire) {
            _nextTimeToFire = Time.time + 1.0f / fireRate;
            Shoot();
            fire = false;
        }


    }


    private void FixedUpdate() {

        _shootingPointVelocity = (shootingPoint.position - _previousShootingPointPosition) / Time.fixedDeltaTime;
        _previousShootingPointPosition = shootingPoint.position;
        
    }



    void Shoot() {
        
        GameObject bullet = _objectPooler.GetPooledObject();
        
        if (bullet != null) {

            Rigidbody rb = bullet.GetComponent<Rigidbody>();
            Projectile projectile = bullet.GetComponent<Projectile>();

            projectile.OnCollisionEnterEvent += DeactivateBullet;
            
            bullet.transform.SetPositionAndRotation(shootingPoint.position, shootingPoint.rotation);

            rb.AddForce(_shootingPointVelocity + shootVelocity * shootingPoint.right, ForceMode.VelocityChange);


            _ = DeactivateBulletAfterLifetime(bullet);

        }
    }

    private void DeactivateBullet(GameObject bullet) {
        _objectPooler.ReturnToPool(bullet);
    }

    private async Task DeactivateBulletAfterLifetime(GameObject bullet) {
        await Task.Delay((int)(maxBulletLifetime * 1000));
        _objectPooler.ReturnToPool(bullet);
    }
}
