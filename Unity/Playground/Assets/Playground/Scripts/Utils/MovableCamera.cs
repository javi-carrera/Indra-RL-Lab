using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MovableCamera : MonoBehaviour {

    public float cameraSpeed = 10.0f;
    public float rotationSpeed = 100.0f;


    // Start is called before the first frame update
    void Start() {
        Cursor.lockState = CursorLockMode.Locked;
        Cursor.visible = false;
        
    }

    // Update is called once per frame
    void Update() {
        HandleMovement();
    }


    
    private void HandleMovement() {
        // Translation
        float horizontal = Input.GetAxis("Horizontal") * cameraSpeed * Time.deltaTime;
        float vertical = Input.GetAxis("Vertical") * cameraSpeed * Time.deltaTime;
        Vector3 rightMovement = transform.right * horizontal;
        Vector3 forwardMovement = transform.forward * vertical;

        transform.position += rightMovement + forwardMovement;

        // Elevation control
        if (Input.GetKey(KeyCode.Q)) {
            transform.position += Vector3.up * cameraSpeed * Time.deltaTime;
        } else if (Input.GetKey(KeyCode.E)) {
            transform.position += Vector3.down * cameraSpeed * Time.deltaTime;
        }

        // Rotation
        if (Input.GetMouseButton(1)) { // Right mouse button

            Debug.Log("Mouse X: " + Input.GetAxis("Mouse X"));

            float yaw = Input.GetAxis("Mouse X") * rotationSpeed * Time.deltaTime;
            float pitch = -Input.GetAxis("Mouse Y") * rotationSpeed * Time.deltaTime;
            
            Vector3 rotation = transform.eulerAngles;
            rotation.x += pitch;
            rotation.y += yaw;
            transform.eulerAngles = rotation;
        }
    }
}
