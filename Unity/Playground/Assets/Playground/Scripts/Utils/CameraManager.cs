using UnityEngine;

public class CameraManager : MonoBehaviour {
    public Camera staticCamera;
    public Camera movableCamera;

    private void Start() {
        staticCamera.enabled = true;
        movableCamera.enabled = false;
    }

    private void Update() {
        if (Input.GetKeyDown(KeyCode.C)) {
            ToggleCamera();
        }
    }


    private void ToggleCamera() {
        staticCamera.enabled = !staticCamera.enabled;
        movableCamera.enabled = !movableCamera.enabled;
    }
}
