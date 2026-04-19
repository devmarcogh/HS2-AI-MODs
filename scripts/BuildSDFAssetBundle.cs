// ============================================================================
// BuildSDFAssetBundle.cs — Unity Editor script to compile compute shaders
// (SDFBuildCompute + ClothSimCompute) into an AssetBundle for BepInEx runtime.
//
// USAGE:
//   1. Create a new empty Unity 2018.4 project (same version as HS2/AI)
//   2. Copy SDFBuildCompute.compute and ClothSimCompute.compute into Assets/Shaders/
//   3. Copy this script into Assets/Editor/
//   4. In Unity: menu → Build → Build SDF Compute Bundle
//   5. Output: Assets/AssetBundles/sdfcompute.unity3d
//   6. Copy sdfcompute.unity3d next to your plugin DLL (BepInEx/plugins/)
// ============================================================================

#if UNITY_EDITOR
using UnityEditor;
using UnityEngine;
using System.IO;

public class BuildSDFAssetBundle
{
    [MenuItem("Build/Build SDF Compute Bundle")]
    public static void Build()
    {
        // Find the compute shader assets
        string sdfShaderPath   = "Assets/Shaders/SDFBuildCompute.compute";
        string clothShaderPath = "Assets/Shaders/ClothSimCompute.compute";

        ComputeShader cs = AssetDatabase.LoadAssetAtPath<ComputeShader>(sdfShaderPath);
        if (cs == null)
        {
            Debug.LogError($"Compute shader not found at {sdfShaderPath}. " +
                           "Place SDFBuildCompute.compute in Assets/Shaders/");
            return;
        }

        ComputeShader clothCs = AssetDatabase.LoadAssetAtPath<ComputeShader>(clothShaderPath);
        if (clothCs == null)
        {
            Debug.LogError($"Compute shader not found at {clothShaderPath}. " +
                           "Place ClothSimCompute.compute in Assets/Shaders/");
            return;
        }

        // Create output directory
        string outputDir = "Assets/AssetBundles";
        if (!Directory.Exists(outputDir))
            Directory.CreateDirectory(outputDir);

        // Tag the assets for the bundle
        AssetImporter importer = AssetImporter.GetAtPath(sdfShaderPath);
        importer.assetBundleName = "sdfcompute";

        AssetImporter clothImporter = AssetImporter.GetAtPath(clothShaderPath);
        clothImporter.assetBundleName = "sdfcompute";

        // Build for Windows standalone (same platform as HS2/AI)
        BuildPipeline.BuildAssetBundles(
            outputDir,
            BuildAssetBundleOptions.None,
            BuildTarget.StandaloneWindows64);

        // Rename to .unity3d for convention
        string src = Path.Combine(outputDir, "sdfcompute");
        string dst = Path.Combine(outputDir, "sdfcompute.unity3d");
        if (File.Exists(dst)) File.Delete(dst);
        if (File.Exists(src)) File.Move(src, dst);

        Debug.Log($"SDF compute bundle built: {dst}");
        Debug.Log("Copy sdfcompute.unity3d next to your plugin DLL in BepInEx/plugins/");
    }
}
#endif
