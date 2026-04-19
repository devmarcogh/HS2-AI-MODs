using System;
using System.IO;
using System.Reflection;
using Unity.Mathematics;
using UnityEngine;

namespace StudioModsMSG
{
    /// <summary>
    /// GPU-accelerated SDF builder using a compute shader.
    /// Falls back to CPU (native or managed) if compute shaders are unavailable
    /// or the AssetBundle containing the shader is not found.
    ///
    /// At 32³–48³ resolution with ~1000 skinned vertices, a brute-force per-voxel
    /// kernel runs in &lt;1ms on any modern GPU — no JFA or spatial hash needed.
    /// </summary>
    struct VertexData
    {
        public Vector3 position;
        public float pad0;
        public Vector3 normal;
        public float pad1;
    }

    struct BoneData
    {
        public int4 indices;
        public float4 weights;
    }

    class GPUSDFBuilder : IDisposable
    {
        // ── Compute shader ──────────────────────────────────────────
        private ComputeShader _shader;
        private int _kernelBuildSDF;
        private int _kernelSkinVertices;
        private int _kernelSmoothSDF;

        // ── GPU Buffers ─────────────────────────────────────────────
        private ComputeBuffer _skinnedVertsBuf;   // VertexData[vertCount]
        private ComputeBuffer _sdfOutputBuf;      // float[totalVoxels] (raw)
        private ComputeBuffer _sdfTempBuf;        // float[totalVoxels] (ping-pong for smoothing)
        private ComputeBuffer _bindVertsBuf;      // BindVertex[vertCount]
        private ComputeBuffer _boneDataBuf;       // BoneData[vertCount]
        private ComputeBuffer _boneMatricesBuf;   // float4x4[boneCount]

        /// <summary>Number of Gaussian blur passes over the SDF. 1-2 recommended.</summary>
        public int SmoothingPasses = 1;

        /// <summary>
        /// The GPU buffer containing the final SDF volume after BuildSDF.
        /// Used by GPUClothSolver for zero-copy SDF collision on GPU.
        /// </summary>
        public ComputeBuffer FinalSDFBuffer { get; private set; }

        // ── Capacities ──────────────────────────────────────────────
        private int _allocVertCount;
        private int _allocVoxelCount;
        private int _allocBoneCount;

        // ── Struct sizes (must match compute shader) ────────────────
        private const int VertexDataStride = 32; // float3 + pad + float3 + pad = 8 floats
        private const int BindVertexStride = 32;
        private const int BoneDataStride   = 32; // int4 + float4 = 8 ints/floats
        private const int Mat4x4Stride     = 64; // 16 floats

        // ── State ───────────────────────────────────────────────────
        private bool _disposed;

        public bool IsReady { get; private set; }

        // ── Asset bundle path (next to the plugin DLL) ──────────────
        private const string ShaderAssetName = "SDFBuildCompute";

        /// <summary>
        /// Try to create a GPU SDF builder. Returns null if compute shaders
        /// are unavailable or the shader asset bundle is missing.
        /// </summary>
        public static GPUSDFBuilder TryCreate()
        {
            if (!SystemInfo.supportsComputeShaders)
                return null;

            ComputeShader cs = LoadComputeShader();
            if (cs == null)
                return null;

            var builder = new GPUSDFBuilder();
            builder._shader = cs;
            builder._kernelBuildSDF = cs.FindKernel("BuildSDF");
            builder._kernelSkinVertices = cs.FindKernel("SkinVertices");
            builder._kernelSmoothSDF = cs.FindKernel("SmoothSDF");
            builder.IsReady = true;
            return builder;
        }

        /// <summary>
        /// Load compute shader from an AssetBundle using reflection
        /// (avoids compile-time dependency on UnityEngine.AssetBundleModule).
        /// </summary>
        private static ComputeShader LoadComputeShader()
        {
            return ComputeBundleLoader.LoadShader(ShaderAssetName);
        }

        /// <summary>
        /// Upload skinned vertex data (positions + normals) to GPU.
        /// Call after CPU skinning, before BuildSDF.
        /// </summary>
        public void UploadSkinnedVertices(Vector3[] positions, float3[] normals, int count)
        {
            EnsureVertBuffers(count);

            VertexData[] data = new VertexData[count];
            for (int i = 0; i < count; i++)
            {
                data[i] = new VertexData {
                    position = positions[i],
                    pad0 = 0f,
                    normal = new Vector3(normals[i].x, normals[i].y, normals[i].z),
                    pad1 = 0f
                };
            }
            _skinnedVertsBuf.SetData(data);
        }

        /// <summary>
        /// Upload bind-pose data for GPU skinning (one-time on init).
        /// </summary>
        public void UploadBindData(float3[] bindPos, float3[] bindNormal,
                                   int4[] boneIdx, float4[] boneW, int count)
        {
            EnsureVertBuffers(count);

            // Bind vertices as VertexData structs
            VertexData[] bindData = new VertexData[count];
            for (int i = 0; i < count; i++)
            {
                bindData[i] = new VertexData {
                    position = new Vector3(bindPos[i].x, bindPos[i].y, bindPos[i].z),
                    pad0 = 0f,
                    normal = new Vector3(bindNormal[i].x, bindNormal[i].y, bindNormal[i].z),
                    pad1 = 0f
                };
            }

            if (_bindVertsBuf == null || _bindVertsBuf.count < count)
            {
                _bindVertsBuf?.Release();
                _bindVertsBuf = new ComputeBuffer(count, BindVertexStride);
            }
            _bindVertsBuf.SetData(bindData);

            // Bone data as BoneData structs (int4 indices + float4 weights)
            BoneData[] boneDataArr = new BoneData[count];
            for (int i = 0; i < count; i++)
            {
                boneDataArr[i] = new BoneData {
                    indices = boneIdx[i],
                    weights = boneW[i]
                };
            }

            if (_boneDataBuf == null || _boneDataBuf.count < count)
            {
                _boneDataBuf?.Release();
                _boneDataBuf = new ComputeBuffer(count, BoneDataStride);
            }
            _boneDataBuf.SetData(boneDataArr);
        }

        /// <summary>
        /// Upload bone matrices for GPU skinning (per frame).
        /// </summary>
        public void UploadBoneMatrices(float4x4[] matrices, int count)
        {
            if (_boneMatricesBuf == null || _allocBoneCount < count)
            {
                _boneMatricesBuf?.Release();
                _boneMatricesBuf = new ComputeBuffer(count, Mat4x4Stride);
                _allocBoneCount = count;
            }

            // float4x4 maps directly to HLSL float4x4 (64 bytes) — no packing needed
            _boneMatricesBuf.SetData(matrices, 0, 0, count);
        }

        /// <summary>
        /// Dispatch GPU skinning. Results stay on GPU for BuildSDF.
        /// </summary>
        public void DispatchSkinVertices(int vertCount)
        {
            _shader.SetInt("_VertCount", vertCount);

            _shader.SetBuffer(_kernelSkinVertices, "_BindVertices", _bindVertsBuf);
            _shader.SetBuffer(_kernelSkinVertices, "_BoneDataBuf", _boneDataBuf);
            _shader.SetBuffer(_kernelSkinVertices, "_BoneMatrices", _boneMatricesBuf);
            _shader.SetBuffer(_kernelSkinVertices, "_SkinnedOutput", _skinnedVertsBuf);

            int groups = (vertCount + 63) / 64;
            _shader.Dispatch(_kernelSkinVertices, groups, 1, 1);
        }

        /// <summary>
        /// Build SDF on GPU. Returns results in outSDF via synchronous readback.
        /// Call UploadSkinnedVertices or DispatchSkinVertices first.
        /// </summary>
        public void BuildSDF(
            int vertCount,
            float3 origin, int resX, int resY, int resZ,
            float cellSize, float maxDist,
            float[] outSDF)
        {
            int totalVoxels = resX * resY * resZ;
            EnsureSDFBuffer(totalVoxels);

            // Set uniforms
            _shader.SetInt("_VertCount", vertCount);
            _shader.SetInt("_ResX", resX);
            _shader.SetInt("_ResY", resY);
            _shader.SetInt("_ResZ", resZ);
            _shader.SetFloat("_OriginX", origin.x);
            _shader.SetFloat("_OriginY", origin.y);
            _shader.SetFloat("_OriginZ", origin.z);
            _shader.SetFloat("_CellSize", cellSize);
            _shader.SetFloat("_MaxDist", maxDist);

            // Bind buffers
            _shader.SetBuffer(_kernelBuildSDF, "_SkinnedVertices", _skinnedVertsBuf);
            _shader.SetBuffer(_kernelBuildSDF, "_SDFOutput", _sdfOutputBuf);

            // Dispatch: numthreads(4,4,4) → groups = ceil(res/4) per axis
            int gx = (resX + 3) / 4;
            int gy = (resY + 3) / 4;
            int gz = (resZ + 3) / 4;
            _shader.Dispatch(_kernelBuildSDF, gx, gy, gz);

            // ── Smoothing passes (ping-pong between _sdfOutputBuf and _sdfTempBuf) ──
            int passes = Math.Max(0, SmoothingPasses);
            ComputeBuffer readBuf  = _sdfOutputBuf;
            ComputeBuffer writeBuf = _sdfTempBuf;

            for (int p = 0; p < passes; p++)
            {
                _shader.SetBuffer(_kernelSmoothSDF, "_SDFInput", readBuf);
                _shader.SetBuffer(_kernelSmoothSDF, "_SDFSmoothed", writeBuf);
                _shader.Dispatch(_kernelSmoothSDF, gx, gy, gz);

                // Swap for next pass
                ComputeBuffer tmp = readBuf;
                readBuf  = writeBuf;
                writeBuf = tmp;
            }

            // Synchronous readback from whichever buffer has final result
            FinalSDFBuffer = readBuf;
            readBuf.GetData(outSDF, 0, 0, totalVoxels);
        }

        /// <summary>
        /// Read back skinned positions + normals to CPU arrays (for AABB computation etc).
        /// </summary>
        public void ReadbackSkinnedVertices(Vector3[] outPos, float3[] outNormal, int count)
        {
            VertexData[] data = new VertexData[count];
            _skinnedVertsBuf.GetData(data, 0, 0, count);

            for (int i = 0; i < count; i++)
            {
                outPos[i]    = data[i].position;
                outNormal[i] = new float3(data[i].normal.x, data[i].normal.y, data[i].normal.z);
            }
        }

        // ── Buffer management ───────────────────────────────────────

        private void EnsureVertBuffers(int count)
        {
            if (_skinnedVertsBuf != null && _allocVertCount >= count) return;

            _skinnedVertsBuf?.Release();
            _skinnedVertsBuf = new ComputeBuffer(count, VertexDataStride);
            _allocVertCount = count;
        }

        private void EnsureSDFBuffer(int totalVoxels)
        {
            if (_sdfOutputBuf != null && _allocVoxelCount >= totalVoxels) return;

            _sdfOutputBuf?.Release();
            _sdfTempBuf?.Release();
            _sdfOutputBuf = new ComputeBuffer(totalVoxels, sizeof(float));
            _sdfTempBuf   = new ComputeBuffer(totalVoxels, sizeof(float));
            _allocVoxelCount = totalVoxels;
        }

        // ── Cleanup ─────────────────────────────────────────────────

        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;

            _skinnedVertsBuf?.Release();
            _sdfOutputBuf?.Release();
            _sdfTempBuf?.Release();
            _bindVertsBuf?.Release();
            _boneDataBuf?.Release();
            _boneMatricesBuf?.Release();

            _skinnedVertsBuf = null;
            _sdfOutputBuf = null;
            _sdfTempBuf = null;
            _bindVertsBuf = null;
            _boneDataBuf = null;
            _boneMatricesBuf = null;
        }
    }
}
