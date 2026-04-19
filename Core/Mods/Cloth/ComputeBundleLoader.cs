using System;
using System.IO;
using System.Reflection;
using UnityEngine;

namespace StudioModsMSG
{
    /// <summary>
    /// Shared AssetBundle loader for compute shaders.
    /// Unity does not allow loading the same AssetBundle twice — if two classes
    /// each call AssetBundle.LoadFromFile on the same file, the second call fails
    /// silently and returns null. This centralises the load so both GPUSDFBuilder
    /// and GPUClothSolver share a single bundle instance.
    /// </summary>
    internal static class ComputeBundleLoader
    {
        private const string BundleName = "sdfcompute.unity3d";

        private static object _cachedBundle;
        private static bool   _loadAttempted;
        private static MethodInfo _loadAssetMethod;

        /// <summary>
        /// Load a ComputeShader by name from the shared AssetBundle.
        /// Returns null if the bundle is missing or the shader is not in it.
        /// </summary>
        public static ComputeShader LoadShader(string shaderAssetName)
        {
            try
            {
                EnsureBundleLoaded();

                if (_cachedBundle == null || _loadAssetMethod == null)
                    return null;

                return _loadAssetMethod.Invoke(_cachedBundle,
                    new object[] { shaderAssetName, typeof(ComputeShader) }) as ComputeShader;
            }
            catch (Exception)
            {
                return null;
            }
        }

        private static void EnsureBundleLoaded()
        {
            if (_loadAttempted) return;
            _loadAttempted = true;

            string dllDir = Path.GetDirectoryName(
                Assembly.GetExecutingAssembly().Location);
            string bundlePath = Path.Combine(dllDir, BundleName);

            if (!File.Exists(bundlePath))
                return;

            // AssetBundle.LoadFromFile(string path) via reflection
            Type abType = Type.GetType("UnityEngine.AssetBundle, UnityEngine")
                       ?? Type.GetType("UnityEngine.AssetBundle, UnityEngine.AssetBundleModule");
            if (abType == null) return;

            MethodInfo loadMethod = abType.GetMethod("LoadFromFile",
                new Type[] { typeof(string) });
            if (loadMethod == null) return;

            _cachedBundle = loadMethod.Invoke(null, new object[] { bundlePath });

            if (_cachedBundle != null)
            {
                _loadAssetMethod = _cachedBundle.GetType().GetMethod("LoadAsset",
                    new Type[] { typeof(string), typeof(Type) });
            }
        }
    }
}
