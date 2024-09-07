using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BillBoard : MonoBehaviour
{
    public new Transform camera;

    private void LateUpdate() {
        transform.LookAt(transform.position + camera.forward);
    }
}
