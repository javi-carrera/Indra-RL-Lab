using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using System.Text;
using Codice.CM.Client.Gui;


[CustomEditor(typeof(AutonomousNavigationExampleEnvironment))]
public class EnvironmentEditor : Editor  {
    
    AutonomousNavigationExampleEnvironment environment;
    public string saveJSONPath = "Assets/Playground/Environments";

    private void OnEnable() {
        environment = (AutonomousNavigationExampleEnvironment)target;
    }

    public override void OnInspectorGUI() {
        
        base.OnInspectorGUI();

        GUILayout.Space(10);
        GUILayout.Label("Environment JSON", EditorStyles.boldLabel); 

        if (GUILayout.Button("Create environment JSON", GUILayout.Width(200))) {
            CreateEnvironmentJson();
        }
    }


    public void CreateEnvironmentJson() {

        environment.Initialize();

        EnvironmentJson environmentJson = new(environment);
        string jsonString = JsonUtility.ToJson(environmentJson, true);
        string agentJSONPath = $"{saveJSONPath}/{environment.environmentName}.json";

        System.IO.File.WriteAllText(agentJSONPath, jsonString);
        Debug.Log($"Agent JSON saved to: {agentJSONPath}");
    }

}
