using UnityEngine;
using UnityEditor;
using System.IO;

public class BuildMenuItems {

    [MenuItem("Build/Build All Platforms")]
    public static void BuildAll() {

        // Build for Windows
        BuildForPlatform(BuildTarget.StandaloneWindows64);

        // Build for Linux
        BuildForPlatform(BuildTarget.StandaloneLinux64);

        Debug.Log("Build process completed successfully!");
    }

    [MenuItem("Build/Build Windows")]
    public static void BuildWindows() {
        BuildForPlatform(BuildTarget.StandaloneWindows64);
    }

    [MenuItem("Build/Build Linux")]
    public static void BuildLinux() {
        BuildForPlatform(BuildTarget.StandaloneLinux64);
    }


    private static void BuildForPlatform(BuildTarget target) {

        string buildFolder = "../builds";
        string platformFolder;
        string extension;


        switch (target) {
            case BuildTarget.StandaloneWindows64:
                platformFolder = "windows";
                extension = ".exe";
                break;
            case BuildTarget.StandaloneLinux64:
                platformFolder = "linux";
                extension = "";
                break;
            default:
                Debug.LogError("Unsupported build target: " + target);
                return;
        }

        // Define the scenes to build
        string[] scenesUC1 = { "Assets/Scenes/UC1Scene.unity" };
        string[] scenesUC2 = { "Assets/Scenes/UC2Scene.unity" };

        // Build UC1 Scene
        BuildPlayerOptions optionsUC1 = new BuildPlayerOptions {
            scenes = scenesUC1,
            locationPathName = Path.Combine(buildFolder, platformFolder, "uc1", "UC1" + extension),
            target = target,
            options = BuildOptions.Development
        };

        BuildPipeline.BuildPlayer(optionsUC1);

        // Build UC2 Scene
        BuildPlayerOptions optionsUC2 = new BuildPlayerOptions {
            scenes = scenesUC2,
            locationPathName = Path.Combine(buildFolder, platformFolder, "uc2", "UC2" + extension),
            target = target,
            options = BuildOptions.Development
        };

        BuildPipeline.BuildPlayer(optionsUC2);
    }
}
