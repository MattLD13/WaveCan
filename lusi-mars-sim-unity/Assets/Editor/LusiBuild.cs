#if UNITY_EDITOR
using System;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEngine;

public static class LusiBuild
{
    private const string ScenePath = "Assets/Scenes/MarsSimulator.unity";
    private const string LinuxBuildPath = "Builds/Linux/LusiMarsSimulator.x86_64";
    private const string WindowsBuildPath = "Builds/Windows/LusiMarsSimulator.exe";

    [MenuItem("LUSI/Build Linux x86_64")]
    public static void BuildLinux()
    {
        ConfigurePlayer();
        PlayerSettings.SetApplicationIdentifier(BuildTargetGroup.Standalone, "com.wavecan.lusi.mars-simulator");
        PlayerSettings.SetScriptingBackend(BuildTargetGroup.Standalone, ScriptingImplementation.IL2CPP);

        if (!EnsureWindowsHostLinuxBuildDependencies())
        {
            EditorApplication.Exit(1);
            return;
        }

        Build(BuildTarget.StandaloneLinux64, LinuxBuildPath);
    }

    [MenuItem("LUSI/Build Windows Test Player")]
    public static void BuildWindowsTest()
    {
        ConfigurePlayer();
        PlayerSettings.SetApplicationIdentifier(BuildTargetGroup.Standalone, "com.wavecan.lusi.mars-simulator");
        PlayerSettings.SetScriptingBackend(BuildTargetGroup.Standalone, ScriptingImplementation.Mono2x);

        Build(BuildTarget.StandaloneWindows64, WindowsBuildPath);
    }

    private static void ConfigurePlayer()
    {
        PlayerSettings.companyName = "WaveCan";
        PlayerSettings.productName = "LUSI Mars Simulator";
        PlayerSettings.defaultScreenWidth = SimulationProfile.ReferenceWidth;
        PlayerSettings.defaultScreenHeight = SimulationProfile.ReferenceHeight;
        PlayerSettings.fullScreenMode = FullScreenMode.Windowed;
        PlayerSettings.runInBackground = true;
    }

    private static bool EnsureWindowsHostLinuxBuildDependencies()
    {
        return InitializeAndVerifyPackage(
                   "UnityEditor.Il2Cpp.SysrootLinuxX86_64, Unity.Sysroot.Linux_x86_64",
                   "GetSysrootPath",
                   "Linux x64 sysroot")
               && InitializeAndVerifyPackage(
                   "UnityEditor.Il2Cpp.ToolchainWindowsX86_64, Unity.Toolchain.Win-x86_64-Linux",
                   "GetToolchainPath",
                   "Windows-host Linux toolchain");
    }

    private static bool InitializeAndVerifyPackage(string typeName, string pathMethodName, string description)
    {
        try
        {
            var packageType = Type.GetType(typeName);
            if (packageType == null)
            {
                Debug.LogError($"LUSI could not load the {description} package type: {typeName}");
                return false;
            }

            var package = Activator.CreateInstance(packageType);
            var initialize = packageType.GetMethod("Initialize");
            var initialized = initialize != null && (bool)initialize.Invoke(package, null);
            var getPath = packageType.GetMethod(pathMethodName);
            var path = getPath == null ? null : getPath.Invoke(package, null) as string;
            var ready = initialized && !string.IsNullOrEmpty(path) && System.IO.Directory.Exists(path);
            Debug.Log($"LUSI {description}: initialized={initialized}; path={path}; ready={ready}");
            return ready;
        }
        catch (Exception exception)
        {
            Debug.LogError($"LUSI could not initialize the {description}: {exception}");
            return false;
        }
    }

    private static void Build(BuildTarget target, string path)
    {
        var options = new BuildPlayerOptions
        {
            scenes = new[] { ScenePath },
            locationPathName = path,
            target = target,
            options = BuildOptions.StrictMode
        };

        var report = BuildPipeline.BuildPlayer(options);
        Debug.Log($"LUSI {target} build result: {report.summary.result}; size={report.summary.totalSize} bytes; path={path}");
        if (report.summary.result != BuildResult.Succeeded)
        {
            EditorApplication.Exit(1);
        }

        if (Application.isBatchMode)
        {
            EditorApplication.Exit(0);
        }
    }
}
#endif
