using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class VectorizedEnvironment : MonoBehaviour {

    public GameObject environment;
    public uint numEnvironments;
    public float x;
    public float y;

    private List<GameObject> _environments;


    void Start() {

        _environments = new List<GameObject>();


        // Get the closest square number to the number of environments
        int numEnvironmentsPerRow = Mathf.CeilToInt(Mathf.Sqrt(numEnvironments));


        // Create the environments
        for (uint i = 0; i < numEnvironments; i++) {

            // Calculate the position of the environment
            float row = i / numEnvironmentsPerRow;
            float col = i % numEnvironmentsPerRow;

            Vector3 position = new(x * col, 0, y * row);

            // Create the environment
            GameObject newEnvironment = Instantiate(environment, position, Quaternion.identity);
            newEnvironment.transform.parent = transform;

            _environments.Add(newEnvironment);

            // Initialize the environment
            newEnvironment.GetComponent<ISingleAgentEnvironment>().Initialize(i);
        }

    }
        


}
