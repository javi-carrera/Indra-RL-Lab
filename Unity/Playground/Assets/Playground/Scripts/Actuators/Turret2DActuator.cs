using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Threading.Tasks;
using RosMessageTypes.InterfacesPkg;

public class Turret2DActuator : Actuator<Turret2DActuatorMsg> {
    
    
    [Header("Turret 2D Actuator Settings")]
    public Transform target;
    public Transform shootingPoint;
    public GameObject bulletPrefab;
    public float rotationSpeed;
    public float shootVelocity;
    public float fireRate;
    [HideInInspector] public float currentAngle;
    
    [HideInInspector] public float targetAngle;
    [HideInInspector] public bool fire;

    private Vector3 _previousShootingPointPosition;
    private Vector3 _shootingPointVelocity;
    private float _cooldown;



    public override void Initialize() {

        // Reset the actuator
        ResetActuator();

    }

    public override void SetActuatorData(Turret2DActuatorMsg msg) {

        // Convert from ROS message to Unity data
        targetAngle = msg.target_angle;
        fire = msg.fire;
    }

    public override void ResetActuator() {

        // Reset ROS message data
        targetAngle = 0.0f;
        fire = false;

        // Reset velocity and cooldown
        _previousShootingPointPosition = shootingPoint.position;
        _shootingPointVelocity = Vector3.zero;
        _cooldown = 1.0f / fireRate;
    }

    protected override void UpdateActuator() {

        // Calculate the target rotation (in global coordinates)
        Quaternion targetRotation = transform.rotation * Quaternion.Euler(0, targetAngle, 0);

        // Rotate the turret towards the target rotation
        target.transform.rotation = Quaternion.RotateTowards(target.transform.rotation, targetRotation, Time.deltaTime * rotationSpeed);

        // Update the current angle
        currentAngle = target.transform.localEulerAngles.y;

        // Fire the bullet
        _cooldown -= Time.deltaTime;
        if (fire && _cooldown <= 0) {
            
            _cooldown = 1.0f / fireRate;
            Shoot();
        }
    }

    private void FixedUpdate() {

        // Calculate the velocity of the shooting point
        _shootingPointVelocity = (shootingPoint.position - _previousShootingPointPosition) / Time.fixedDeltaTime;
        _previousShootingPointPosition = shootingPoint.position;
        
    }

    private void Shoot() {
        
        // GameObject bullet = _objectPooler.GetPooledObject();
        GameObject bullet = Instantiate(bulletPrefab);
        
        if (bullet != null) {

            // Get the rigidbody of the bullet
            Rigidbody rb = bullet.GetComponent<Rigidbody>();
            
            // Set the position and rotation of the bullet
            bullet.transform.SetPositionAndRotation(shootingPoint.position, shootingPoint.rotation);

            // Set the velocity of the bullet
            rb.AddForce(_shootingPointVelocity + shootVelocity * shootingPoint.forward, ForceMode.VelocityChange);

        }
    }
}
