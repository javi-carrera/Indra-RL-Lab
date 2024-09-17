using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Sensor;
using RosMessageTypes.InterfacesPkg;


[System.Serializable]
public struct TagData {
    public string tagName;
    public int tagNumber;

    public TagData(string name, int number) {
        tagName = name;
        tagNumber = number;
    }
}

public class SmartLidar2DSensor : Sensor {
    
    [HideInInspector] public SmartLaserScanMsg smartLaserScan2DMsg;

    [Header("Smart Lidar Sensor Settings")]
    public Vector3 positionOffset;
    public float angleMin;
    public float angleMax;
    public uint numRays;
    public float rangeMin;
    public float rangeMax;
    public List<TagData> tagsData;
    private float[] _ranges;
    private int[] _tags;
    private float _angleIncrement;

    [Header("Debug Settings")]
    public bool drawDebugRays;
    private Color _hitColor = Color.red;
    private Color _noHitColor = Color.green;



    public override void Initialize() {
        smartLaserScan2DMsg = new SmartLaserScanMsg();
    }

    public override void GetSensorData() {

        // Convert Unity data to ROS message
        smartLaserScan2DMsg.angle_min = angleMin;
        smartLaserScan2DMsg.angle_max = angleMax;
        smartLaserScan2DMsg.angle_increment = _angleIncrement;
        smartLaserScan2DMsg.range_min = rangeMin;
        smartLaserScan2DMsg.range_max = rangeMax;
        smartLaserScan2DMsg.ranges = _ranges;
        smartLaserScan2DMsg.tags = _tags;

    }

    protected override void UpdateSensor() {

        // Check the number of rays is greater than 0
        if (numRays <= 0) {
            Debug.LogError("numRays must be greater than 0");
            return;
        }
        
        // Initialize ranges array and calculate angle increment
        _ranges = new float[numRays];
        _tags = new int[numRays];
        _angleIncrement = numRays == 1 ? (angleMax - angleMin) : (angleMax - angleMin) / (numRays - 1);
        
        // Check if rangeMax is greater than rangeMin
        if (rangeMax <= rangeMin) {

            Debug.LogError("rangeMax must be greater than rangeMin");

            // Initialize ranges array with 0s and tags array with -1s
            for (int i = 0; i < numRays; i++) {
                _ranges[i] = 0.0f;
                _tags[i] = -1;
            }

            return;
        }



        for (int i = 0; i < numRays; i++) {

            // Calculate ray angle, origin and direction
            float angle = angleMin + i * _angleIncrement;
            Vector3 rotation = transform.TransformDirection(Quaternion.Euler(0, -angle, 0) * Vector3.right);
            Vector3 origin = transform.position + positionOffset + rotation * rangeMin;
            Vector3 direction = rotation ;

            // Cast ray and get distance
            RaycastHit hit;
            if (Physics.Raycast(origin, direction, out hit, rangeMax - rangeMin, ~LayerMask.GetMask("TriggerVolume"))) {
                _ranges[i] = hit.distance + rangeMin;
                _tags[i] = tagsData.FindIndex(tag => hit.collider.CompareTag(tag.tagName));
            }
            else {
                _ranges[i] = rangeMax;
                _tags[i] = -1;
            }

            // Draw debug rays
            if (drawDebugRays) {
                Color color = hit.collider ? _hitColor : _noHitColor;
                Debug.DrawRay(origin, direction * (_ranges[i] - rangeMin), color);
            }
        }
    }



    public override void ResetSensor() {
    }

}
