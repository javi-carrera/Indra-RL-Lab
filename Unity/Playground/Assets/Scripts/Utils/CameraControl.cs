using UnityEngine;

public class CameraControl : MonoBehaviour {
    public float dampTime = 0.2f;                 
    public float screenEdgeBuffer = 4f;           
    public float minSize = 6.5f;                  
    public Transform[] targets; 


    private Camera _camera;                        
    private float _zoomSpeed;                      
    private Vector3 _moveVelocity;                 
    private Vector3 _desiredPosition;              


    private void Awake() {
        _camera = GetComponentInChildren<Camera>();
    }


    private void FixedUpdate() {
        Move();
        Zoom();
    }


    private void Move() {

        FindAveragePosition();

        transform.position = Vector3.SmoothDamp(transform.position, _desiredPosition, ref _moveVelocity, dampTime);
    }


    private void FindAveragePosition() {


        Vector3 averagePos = new Vector3();
        int numTargets = 0;

        for (int i = 0; i < targets.Length; i++) {

            if (!targets[i].gameObject.activeSelf)
                continue;

            averagePos += targets[i].position;
            numTargets++;
        }

        if (numTargets > 0)
            averagePos /= numTargets;

        averagePos.y = transform.position.y;

        _desiredPosition = averagePos;
    }


    private void Zoom() {
        float requiredSize = FindRequiredSize();
        _camera.orthographicSize = Mathf.SmoothDamp(_camera.orthographicSize, requiredSize, ref _zoomSpeed, dampTime);
    }


    private float FindRequiredSize() {
        Vector3 desiredLocalPos = transform.InverseTransformPoint(_desiredPosition);

        float size = 0f;

        for (int i = 0; i < targets.Length; i++)
        {
            if (!targets[i].gameObject.activeSelf)
                continue;

            Vector3 targetLocalPos = transform.InverseTransformPoint(targets[i].position);

            Vector3 desiredPosToTarget = targetLocalPos - desiredLocalPos;

            size = Mathf.Max (size, Mathf.Abs (desiredPosToTarget.y));

            size = Mathf.Max (size, Mathf.Abs (desiredPosToTarget.x) / _camera.aspect);
        }
        
        size += screenEdgeBuffer;

        size = Mathf.Max(size, minSize);

        return size;
    }


    public void SetStartPositionAndSize() {

        FindAveragePosition();

        transform.position = _desiredPosition;

        _camera.orthographicSize = FindRequiredSize();
    }
}