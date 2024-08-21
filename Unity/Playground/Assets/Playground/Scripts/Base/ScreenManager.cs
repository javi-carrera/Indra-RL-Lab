using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ScreenManager : MonoBehaviour {
    

    public uint width = 1280;
    public uint height = 720;
    private int _currentWidth;
    private int _currentHeight;

    private void Start() {
        // Initialize with the current screen resolution
        _currentWidth = (int)width;
        _currentHeight = (int)height;
        SetScreenResolution(_currentWidth, _currentHeight);
    }

    private void Update() {
        // Check every frame to see if the window size has changed
        if (_currentWidth != Screen.width || _currentHeight != Screen.height) {
            _currentWidth = Screen.width;
            _currentHeight = Screen.height;
            SetScreenResolution(_currentWidth, _currentHeight);
        }
    }

    private void SetScreenResolution(int width, int height) {
        // Set the screen resolution and update the internal state
        Screen.SetResolution(width, height, false);
        Debug.Log($"Resolution set to: {width}x{height}");
    }
}
