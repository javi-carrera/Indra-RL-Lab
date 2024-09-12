// Project: Playground
// File: CameraSensor.cs
// Authors: Javier Carrera, Guillermo Escolano
// License: Apache 2.0 (refer to LICENSE file in the project root)

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Sensor;
using System.IO;
using System;

public class CameraSensor : Sensor {

    [HideInInspector] public CompressedImageMsg compressedImage;

    [Header("Camera Sensor Settings")]
    public Camera targetCamera;

    private Texture2D _texture2D;
    byte[] imageData;

    public override void Initialize() {
        // Initialize the ROS message
        compressedImage = new CompressedImageMsg();
        
    }

    public override void GetSensorData() {
        // Fill the CompressedImage message
        compressedImage.format = "png";  // Define format
        compressedImage.data = imageData;
    }

    protected override void UpdateSensor() {

        _texture2D = new Texture2D(targetCamera.pixelWidth, targetCamera.pixelHeight, TextureFormat.RGB24, false);

        // Capture the image from the target camera
        RenderTexture renderTexture = new RenderTexture(targetCamera.pixelWidth, targetCamera.pixelHeight, 24);
        targetCamera.targetTexture = renderTexture;
        targetCamera.Render();

        // Read the pixels from the render texture
        RenderTexture.active = renderTexture;
        _texture2D.ReadPixels(new Rect(0, 0, targetCamera.pixelWidth, targetCamera.pixelHeight), 0, 0);
        _texture2D.Apply();

        // Convert the texture to a byte array (PNG format as an example)
        imageData = _texture2D.EncodeToPNG();

        // Clean up
        targetCamera.targetTexture = null;
        RenderTexture.active = null;
        Destroy(renderTexture);
    }

    public override void ResetSensor() {
        // Add any reset logic here if needed
    }
}


