#if UNITY_EDITOR
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;

[InitializeOnLoad]
public static class LusiProjectSetup
{
    private const string PipelineAssetPath = "Assets/Settings/LusiUniversalRenderPipeline.asset";
    private const string RendererAssetPath = "Assets/Settings/LusiForwardRenderer.asset";

    static LusiProjectSetup()
    {
        EditorApplication.delayCall += EnsurePipelineAsset;
    }

    private static void EnsurePipelineAsset()
    {
        if (EditorApplication.isCompiling || EditorApplication.isUpdating)
        {
            return;
        }

        EnsureFolder("Assets/Settings");
        var pipeline = AssetDatabase.LoadAssetAtPath<UniversalRenderPipelineAsset>(PipelineAssetPath);
        if (pipeline == null || pipeline.name != "Lusi Deck URP" || pipeline.rendererDataList.Length == 0 || pipeline.rendererDataList[0] == null)
        {
            if (pipeline != null)
            {
                AssetDatabase.DeleteAsset(PipelineAssetPath);
            }

            var renderer = AssetDatabase.LoadAssetAtPath<UniversalRendererData>(RendererAssetPath);
            if (renderer == null)
            {
                renderer = ScriptableObject.CreateInstance<UniversalRendererData>();
                renderer.name = "LUSI Forward Renderer";
                AssetDatabase.CreateAsset(renderer, RendererAssetPath);
            }

            pipeline = UniversalRenderPipelineAsset.Create(renderer);
            pipeline.name = "LUSI Deck URP";
            AssetDatabase.CreateAsset(pipeline, PipelineAssetPath);
            AssetDatabase.SaveAssets();
        }

        GraphicsSettings.defaultRenderPipeline = pipeline;
        QualitySettings.renderPipeline = pipeline;
        AssignPipelineToQualityLevels(pipeline);
        EnsureRuntimeShadersIncluded();
        EditorUtility.SetDirty(pipeline);
        AssetDatabase.SaveAssets();
    }

    private static void AssignPipelineToQualityLevels(UniversalRenderPipelineAsset pipeline)
    {
        var qualitySettings = AssetDatabase.LoadMainAssetAtPath("ProjectSettings/QualitySettings.asset");
        if (qualitySettings == null)
        {
            return;
        }

        var serialized = new SerializedObject(qualitySettings);
        var levels = serialized.FindProperty("m_QualitySettings");
        for (var index = 0; index < levels.arraySize; index++)
        {
            levels.GetArrayElementAtIndex(index).FindPropertyRelative("customRenderPipeline").objectReferenceValue = pipeline;
        }

        serialized.ApplyModifiedPropertiesWithoutUndo();
    }

    private static void EnsureRuntimeShadersIncluded()
    {
        var graphicsSettings = GraphicsSettings.GetGraphicsSettings();
        if (graphicsSettings == null)
        {
            return;
        }

        var serialized = new SerializedObject(graphicsSettings);
        var alwaysIncluded = serialized.FindProperty("m_AlwaysIncludedShaders");
        if (alwaysIncluded == null)
        {
            return;
        }

        var requiredNames = new[]
        {
            "LUSI/MarsRegolith",
            "Universal Render Pipeline/Lit",
            "Universal Render Pipeline/Unlit"
        };

        foreach (var shaderName in requiredNames)
        {
            var shader = Shader.Find(shaderName);
            var alreadyIncluded = false;
            for (var index = 0; index < alwaysIncluded.arraySize; index++)
            {
                if (alwaysIncluded.GetArrayElementAtIndex(index).objectReferenceValue == shader)
                {
                    alreadyIncluded = true;
                    break;
                }
            }

            if (shader != null && !alreadyIncluded)
            {
                alwaysIncluded.arraySize++;
                alwaysIncluded.GetArrayElementAtIndex(alwaysIncluded.arraySize - 1).objectReferenceValue = shader;
            }
        }

        serialized.ApplyModifiedPropertiesWithoutUndo();
    }

    private static void EnsureFolder(string path)
    {
        if (!AssetDatabase.IsValidFolder(path))
        {
            AssetDatabase.CreateFolder("Assets", "Settings");
        }
    }
}
#endif
