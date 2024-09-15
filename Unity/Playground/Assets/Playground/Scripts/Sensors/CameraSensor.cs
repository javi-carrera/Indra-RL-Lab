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
    private RenderTexture _renderTexture;
    private byte[] _imageData;

    public override void Initialize() {

        // Initialize the ROS message
        compressedImage = new CompressedImageMsg();
        compressedImage.format = "png";

        // Create a new texture to store the image
        _texture2D = new Texture2D(targetCamera.pixelWidth, targetCamera.pixelHeight, TextureFormat.RGB24, false);

        // Create a new render texture to capture the image
        _renderTexture = new RenderTexture(targetCamera.pixelWidth, targetCamera.pixelHeight, 24);

    }

    public override void GetSensorData() {
        
        // Encode the texture to PNG
        _imageData = _texture2D.EncodeToPNG();

        // Fill the CompressedImage message
        compressedImage.data = _imageData;
    }

    protected override void UpdateSensor() {

        if (_renderTexture.width != targetCamera.pixelWidth || _renderTexture.height != targetCamera.pixelHeight) {

            // Update the render texture size
            _renderTexture.Release();
            _renderTexture.width = targetCamera.pixelWidth;
            _renderTexture.height = targetCamera.pixelHeight;

            // Update the texture size
            _texture2D.Reinitialize(targetCamera.pixelWidth, targetCamera.pixelHeight);

        }

        // Capture the image from the target camera
        targetCamera.targetTexture = _renderTexture;
        targetCamera.Render();

        // Read the pixels from the render texture
        RenderTexture.active = _renderTexture;
        _texture2D.ReadPixels(new Rect(0, 0, targetCamera.pixelWidth, targetCamera.pixelHeight), 0, 0);
        _texture2D.Apply();

        // Clean up
        targetCamera.targetTexture = null;
        RenderTexture.active = null;
    }

    public override void ResetSensor() {
        // Add any reset logic here if needed
    }
}


