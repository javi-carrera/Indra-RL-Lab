using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;


public class SceneLoader : MonoBehaviour {

    public List<string> sceneNames;

    void Update() {
        for (int i = 0; i < sceneNames.Count; i++) {
            if (Input.GetKeyDown(KeyCode.Alpha1 + i)) {
                LoadScene(sceneNames[i]);
            }
        }
    }

    public static void LoadScene(string sceneName) {
        SceneManager.LoadScene(sceneName);
    }
}
