using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Sensor;

public class LidarSensor : Sensor {
    
    public Vector3 positionOffset;
    public float angleMin;
    public float angleMax;
    public uint numRays;
    public float rangeMax;
    public float rangeMin;
    public bool drawDebugRays;

    [HideInInspector]
    public LaserScanMsg laserScan;
    private float[] _ranges;
    private Color _hitColor = Color.red;
    private Color _noHitColor = Color.green;
    private float _angleIncrement;


    void Update() {
        
        // Initialize ranges array and calculate angle increment
        _ranges = new float[numRays];
        _angleIncrement = (angleMax - angleMin) / (numRays - 1);
        
        // Check if rangeMax is greater than rangeMin
        if (rangeMax <= rangeMin) {

            Debug.LogError("rangeMax must be greater than rangeMin");

            // Initialize ranges array with 0s
            for (int i = 0; i < numRays; i++) {
                _ranges[i] = 0;
            }
            
            return;
        }

        for (int i = 0; i < numRays; i++) {

            // Calculate ray angle, origin and direction
            float angle = angleMin + i * _angleIncrement;
            Vector3 origin = transform.position + positionOffset + Quaternion.Euler(0, angle, 0) * transform.forward * rangeMin;
            Vector3 direction = Quaternion.Euler(0, angle, 0) * transform.forward;

            // Cast ray and get distance
            RaycastHit hit;
            if (Physics.Raycast(origin, direction, out hit, rangeMax - rangeMin)) {
                _ranges[i] = hit.distance + rangeMin;
            }
            else {
                _ranges[i] = rangeMax;
            }

            // Draw debug rays
            if (drawDebugRays) {
                Color color = hit.collider ? _hitColor : _noHitColor;
                Debug.DrawRay(origin, direction * (_ranges[i] - rangeMin), color);
            }
        }
    }

    public override void GetData() {

        // Convert Unity data to ROS message
        laserScan.angle_min = angleMin;
        laserScan.angle_max = angleMax;
        laserScan.angle_increment = _angleIncrement;
        laserScan.range_min = rangeMin;
        laserScan.range_max = rangeMax;
        laserScan.ranges = _ranges;

    }
}
