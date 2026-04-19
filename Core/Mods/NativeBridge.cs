using System;
using System.Runtime.InteropServices;
using UnityEngine;

namespace StudioModsMSG
{
    /// <summary>
    /// P/Invoke bridge to StudioModsNative.dll.
    /// Provides SIMD-accelerated PBD solving, spatial hashing, and batch skinning.
    /// Falls back to managed code if the native DLL is not present.
    /// </summary>
    static class NativeBridge
    {
        private const string DLL = "StudioModsNative";

        private static bool? _available;

        /// <summary>
        /// Whether the native DLL is loaded and usable.
        /// Cached after first check.
        /// </summary>
        public static bool IsAvailable
        {
            get
            {
                if (!_available.HasValue)
                {
                    try
                    {
                        // Probe by calling a trivial function
                        long h = PBD_Create(0, new int[0], new float[0], 0, new int[0], new int[0], new float[0]);
                        PBD_Destroy(h);
                        _available = true;
                    }
                    catch (DllNotFoundException)
                    {
                        _available = false;
                    }
                    catch (EntryPointNotFoundException)
                    {
                        _available = false;
                    }
                }
                return _available.Value;
            }
        }

        // ── Interop structs (must match C++ layout) ─────────────

        [StructLayout(LayoutKind.Sequential)]
        public struct Vec3Native
        {
            public float x, y, z;

            public Vec3Native(Vector3 v) { x = v.x; y = v.y; z = v.z; }
            public Vector3 ToVector3() => new Vector3(x, y, z);
        }

        [StructLayout(LayoutKind.Sequential)]
        public struct BoneWeightNative
        {
            public int idx0, idx1, idx2, idx3;
            public float w0, w1, w2, w3;

            public BoneWeightNative(BoneWeight bw)
            {
                idx0 = bw.boneIndex0; idx1 = bw.boneIndex1;
                idx2 = bw.boneIndex2; idx3 = bw.boneIndex3;
                w0 = bw.weight0; w1 = bw.weight1;
                w2 = bw.weight2; w3 = bw.weight3;
            }
        }

        [StructLayout(LayoutKind.Sequential)]
        public struct Mat4x4Native
        {
            // 16 floats, column-major (same as Unity Matrix4x4)
            public float m00, m10, m20, m30;
            public float m01, m11, m21, m31;
            public float m02, m12, m22, m32;
            public float m03, m13, m23, m33;

            public Mat4x4Native(Matrix4x4 m)
            {
                m00 = m.m00; m10 = m.m10; m20 = m.m20; m30 = m.m30;
                m01 = m.m01; m11 = m.m11; m21 = m.m21; m31 = m.m31;
                m02 = m.m02; m12 = m.m12; m22 = m.m22; m32 = m.m32;
                m03 = m.m03; m13 = m.m13; m23 = m.m23; m33 = m.m33;
            }
        }

        // ── PBD Solver ──────────────────────────────────────────

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern long PBD_Create(
            int sbVertexCount,
            [In] int[] sbVertexIndices,
            [In] float[] inverseMass,
            int edgeCount,
            [In] int[] edgeA,
            [In] int[] edgeB,
            [In] float[] edgeRestLength
        );

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern void PBD_Step(
            long handle,
            [In, Out] float[] physWorldPos,
            [In, Out] float[] velocity,
            [In] float[] animatedPos,
            float dt,
            float intensity,
            float stiffness,
            float damping,
            float gravitySag,
            float lateralMul,
            float secondaryAmp,
            float refAccelX, float refAccelY, float refAccelZ,
            float lateralDirX, float lateralDirY, float lateralDirZ,
            float time,
            int iterations
        );

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern void PBD_Destroy(long handle);

        // ── Spatial Hash ────────────────────────────────────────

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern long SHash_Create(int capacity);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern void SHash_Build(
            long handle,
            [In] float[] positions,
            int count,
            float cellSize
        );

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern int SHash_Query(
            long handle,
            float cx, float cy, float cz,
            float radius,
            [Out] int[] outIndices,
            int maxResults
        );

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern void SHash_Destroy(long handle);

        // ── Batch Skinning ──────────────────────────────────────

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern void Skin_ForwardBatch(
            [In] Vec3Native[] bindVertices,
            [In] BoneWeightNative[] boneWeights,
            [In] Mat4x4Native[] skinMatrices,
            [In] int[] sbVertexIndices,
            int sbCount,
            [Out] float[] outWorldPos
        );

        // ── Managed helpers for Vector3[] <-> float[] ───────────

        public static float[] Vec3ArrayToFlat(Vector3[] src, int count)
        {
            float[] flat = new float[count * 3];
            for (int i = 0; i < count; i++)
            {
                flat[i * 3]     = src[i].x;
                flat[i * 3 + 1] = src[i].y;
                flat[i * 3 + 2] = src[i].z;
            }
            return flat;
        }

        public static void FlatToVec3Array(float[] flat, Vector3[] dst, int count)
        {
            for (int i = 0; i < count; i++)
            {
                dst[i].x = flat[i * 3];
                dst[i].y = flat[i * 3 + 1];
                dst[i].z = flat[i * 3 + 2];
            }
        }

        public static Vec3Native[] ToNativeVec3(Vector3[] src)
        {
            var dst = new Vec3Native[src.Length];
            for (int i = 0; i < src.Length; i++)
                dst[i] = new Vec3Native(src[i]);
            return dst;
        }

        public static BoneWeightNative[] ToNativeBoneWeights(BoneWeight[] src)
        {
            var dst = new BoneWeightNative[src.Length];
            for (int i = 0; i < src.Length; i++)
                dst[i] = new BoneWeightNative(src[i]);
            return dst;
        }

        public static Mat4x4Native[] ToNativeMat4x4(Matrix4x4[] src)
        {
            var dst = new Mat4x4Native[src.Length];
            for (int i = 0; i < src.Length; i++)
                dst[i] = new Mat4x4Native(src[i]);
            return dst;
        }

        // ── Cloth xPBD Solver ───────────────────────────────────

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern long Cloth_Create(int vertCount, int edgeCount, int bendCount);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern void Cloth_Destroy(long handle);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern void Cloth_Predict(
            [In, Out] float[] pos, [In, Out] float[] vel, [Out] float[] pred,
            [In] float[] invMass, int n, float gravity, float dt);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern void Cloth_SolveEdges(
            [In, Out] float[] pred, [In] float[] invMass, [In] int[] isPinned,
            [In] int[] edges, [In] float[] restLen, int edgeCount,
            float tildedCompliance, float pressScale);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern void Cloth_SolveBends(
            [In, Out] float[] pred, [In] float[] invMass, [In] int[] isPinned,
            [In] int[] bendPairs, [In] float[] restBendLen, int bendCount,
            float tildedCompliance);

        [StructLayout(LayoutKind.Sequential)]
        public struct NativeCollider
        {
            public float cx, cy, cz;
            public float radius;
            public int   isCapsule;
            public float p1x, p1y, p1z;
            public float vx, vy, vz;
            public float vDotV;
            public int   magneticMode;
            public float magneticStrength;
            public float magneticRange;
        }

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern void Cloth_SolveCollision(
            [In, Out] float[] pred, [In] float[] invMass, int n,
            [In] NativeCollider[] colliders, int colCount, float thickness);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern void Cloth_Commit(
            [In, Out] float[] pos, [Out] float[] vel, [In] float[] pred,
            [In] float[] invMass, int n, float invDt, float damp, float maxSpeed);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern void Cloth_SkinVertices(
            [In] float[] bindVerts, [In] int[] boneIdx, [In] float[] boneW,
            [In] float[] skinMats, int n, [Out] float[] outWorld);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern void Cloth_WorldToLocal(
            [In] float[] worldPos, [In] float[] w2lMat, int n, [Out] float[] localPos);

        // ── SDF Collider ────────────────────────────────────────

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern long SDF_Create(int maxVoxels, int maxVertices);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern void SDF_Destroy(long handle);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern void SDF_SkinVertices(
            [In] float[] bindPos, [In] float[] bindNormal,
            [In] int[] boneIdx, [In] float[] boneW,
            [In] float[] boneMats, int count,
            [Out] float[] outPos, [Out] float[] outNormal);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern void SDF_Build(
            long handle,
            [In] float[] skinnedPos, [In] float[] skinnedNormal, int vertCount,
            [In] int[] indices, int indexCount,
            float originX, float originY, float originZ,
            int resX, int resY, int resZ,
            float cellSize, float maxDist,
            float hashCellSize, float queryRadius,
            [Out] float[] outSDF);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern float SDF_Sample(
            [In] float[] sdfData,
            int resX, int resY, int resZ,
            float originX, float originY, float originZ,
            float invCellSize, float maxDist,
            float wx, float wy, float wz);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern void SDF_CollideVertices(
            [In, Out] float[] pred, [In] float[] invMass, int n,
            [In] float[] sdfData,
            int resX, int resY, int resZ,
            float originX, float originY, float originZ,
            float invCellSize, float maxDist,
            float thickness);

        // ── VBD Cloth Solver ────────────────────────────────────

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern long Cloth_CreateVBD(
            int vertCount,
            [In] float[] pos,
            [In] float[] invMass,
            int edgeCount,
            [In] int[] edgeIndices,
            [In] float[] restLen,
            int bendCount,
            [In] int[] bendIndices,
            [In] float[] restBendLen);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern void Cloth_StepVBD(
            long handle,
            [In, Out] float[] pos,
            [In, Out] float[] vel,
            [In] NativeCollider[] colliders,
            int colCount,
            [In] float[] sdfData,
            int sdfResX, int sdfResY, int sdfResZ,
            float sdfOriginX, float sdfOriginY, float sdfOriginZ,
            float sdfInvCellSize, float sdfMaxDist, float sdfThickness,
            float dt,
            int substeps,
            int iterations,
            float gravity,
            float stretchStiffness,
            float bendStiffness,
            float damping,
            float friction,
            float thickness,
            float maxSpeed,
            float compression,
            [In] float[] invMass);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern void Cloth_DestroyVBD(long handle);

        // ── Managed helpers for bool[] -> int[] ─────────────────

        public static int[] BoolToInt(bool[] src, int count)
        {
            int[] dst = new int[count];
            for (int i = 0; i < count; i++)
                dst[i] = src[i] ? 1 : 0;
            return dst;
        }

        public static float[] Matrix4x4ToFlat(Matrix4x4 m)
        {
            return new float[] {
                m.m00, m.m10, m.m20, m.m30,
                m.m01, m.m11, m.m21, m.m31,
                m.m02, m.m12, m.m22, m.m32,
                m.m03, m.m13, m.m23, m.m33
            };
        }

        public static float[] Matrix4x4ArrayToFlat(Matrix4x4[] src, int count)
        {
            float[] flat = new float[count * 16];
            for (int i = 0; i < count; i++)
            {
                Matrix4x4 m = src[i];
                int off = i * 16;
                flat[off]      = m.m00; flat[off + 1]  = m.m10; flat[off + 2]  = m.m20; flat[off + 3]  = m.m30;
                flat[off + 4]  = m.m01; flat[off + 5]  = m.m11; flat[off + 6]  = m.m21; flat[off + 7]  = m.m31;
                flat[off + 8]  = m.m02; flat[off + 9]  = m.m12; flat[off + 10] = m.m22; flat[off + 11] = m.m32;
                flat[off + 12] = m.m03; flat[off + 13] = m.m13; flat[off + 14] = m.m23; flat[off + 15] = m.m33;
            }
            return flat;
        }
    }
}
