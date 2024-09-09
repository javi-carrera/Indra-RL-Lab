using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DroneCameraSight : MonoBehaviour
{

    public Material FoundMaterial;
    public Material NotFoundMaterial;

    private bool playerInSight = false;
    private Renderer objectRenderer;
    // Start is called before the first frame update
    void Start()
    {

        objectRenderer = gameObject.GetComponent<Renderer>();
        objectRenderer.material = NotFoundMaterial;
    }

    private void OnTriggerEnter(Collider other) {
        if (other.gameObject.tag == "tank") {
            Debug.Log("Player is in sight");

            objectRenderer.material = FoundMaterial;

        }
    }

    private void OnTriggerExit(Collider other) {
        if (other.gameObject.tag == "tank") {
            Debug.Log("Player is out of sight");

            objectRenderer.material = NotFoundMaterial;
            
        }
    }
}
