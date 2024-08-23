using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;

public class UIManager : MonoBehaviour {

    public GameObject environment;
    public TextMeshProUGUI pauseText;
    public TextMeshProUGUI sampleTimeText;
    public TextMeshProUGUI timeScaleText;
    public TextMeshProUGUI rosPortText;


    protected void Update() {

        IEnvironment environmentScript = environment.GetComponent<IEnvironment>();

        if (pauseText != null) pauseText.text = $"Pause: {environmentScript.Pause}";
        if (sampleTimeText != null) sampleTimeText.text = "Sample Time: " + environmentScript.SampleTime.ToString("F2") + " seconds";
        if (timeScaleText != null) timeScaleText.text = "Time Scale: " + environmentScript.TimeScale.ToString("F1");
        if (rosPortText != null) rosPortText.text = "ROS Port: " + environmentScript.RosPort;

    }
}
