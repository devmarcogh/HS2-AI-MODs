using System;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using Unity.Mathematics;
using UnityEngine;

namespace StudioModsMSG
{
    /// <summary>
    /// SDF-based body proxy collider using vertex-distance approach.
    ///
    /// Algorithm (CPU, per frame):
    ///   1. Skin decimated vertices via 4-bone LBS (Unity.Mathematics, Parallel.For)
    ///   2. Transform vertex normals via bone matrices (parallel)
    ///   3. Build spatial hash of skinned vertex positions
    ///   4. For each voxel (Parallel.For): find closest vertex via spatial hash,
    ///      compute signed distance using vertex normal for sign determination
    ///   5. O(1) collision queries via trilinear interpolation + central-difference gradient
    ///
    /// Advantages over capsule-only collision:
    ///   - Continuous collision surface (no gaps between particles)
    ///   - O(1) distance query per cloth vertex (no spatial hash lookup per vertex)
    ///   - Free surface normal from SDF gradient (central differences)
    ///
    /// Performance vs old triangle-splatting approach:
    ///   - Vertex-distance is O(voxels) with full parallelism
    ///   - Triangle-splatting was O(triangles × voxel_neighborhood) sequential
    ///   - Decimation reduces skinning cost to ~1000 vertices
    ///   - Spatial hash makes nearest-vertex lookup O(1) per voxel
    /// </summary>
    public class SDFBodyCollider : MonoBehaviour
    {
        [Header("SDF Configuration")]
        [Tooltip("1 = every vertex, 2 = half, etc. Lower = more accurate SDF but slower build.")]
        public int decimationFactor = 3;

        [Tooltip("Max voxel grid resolution per axis. 24-32 recommended.")]
        public int resolution = 32;

        [Tooltip("Extra padding around the region bounding box (world units).")]
        public float padding = 0.06f;

        [Tooltip("Collision surface offset from body (metres). Should be >= Cloth Thickness.")]
        public float surfaceOffset = 0.05f;

        [Tooltip("Rebuild SDF every N frames. 2-3 saves CPU with minimal lag.")]
        public int updateInterval = 1;

        [Tooltip("Gradient sampling radius in cell-size units. Lower keeps shape tighter; higher smooths jitter.")]
        public float gradientSampleFactor = 1.5f;

        // ── Skinning data (decimated vertices) ──────────────────────────
        private float3[]    _bindLocal;     // rest-pose positions  (mesh local)
        private float3[]    _bindNormal;    // rest-pose normals    (mesh local)
        private int4[]      _boneIdx;       // 4 bone indices per vertex
        private float4[]    _boneW;         // 4 bone weights per vertex
        private float4x4[]  _boneMatrices;  // world-space skinning matrices
        private Transform[] _bones;
        private Matrix4x4[] _bindPoses;
        private int         _vertCount;

        // ── Per-frame skinned output ────────────────────────────────────
        private Vector3[]   _skinnedPos;    // world-space (Vector3 for SpatialHashGrid compat)
        private float3[]    _skinnedNormal; // world-space normals (for sign determination)

        // ── SDF volume ─────────────────────────────────────────────────
        private float[]     _sdfData;
        private int         _resX, _resY, _resZ;
        private int         _totalVoxels;
        private float3      _origin;        // world-space corner of voxel (0,0,0)
        private float       _cellSizeActual;
        private float       _invCellSize;
        private float       _maxDist;       // sentinel for "no data" voxels


        // ── Frame throttle ─────────────────────────────────────────────
        private int _frameCounter;

        // ── Native acceleration ────────────────────────────────────────
        private bool   _nativeChecked;
        private bool   _nativeAvailable;
        private long   _nativeSDFHandle;
        private float[] _nBindPos, _nBindNormal;
        private int[]   _nBoneIdx;
        private float[] _nBoneW;
        private float[] _nBoneMatsFlat;
        private float[] _nSkinnedPos, _nSkinnedNormal;

        // ── GPU acceleration ───────────────────────────────────────────
        private GPUSDFBuilder _gpuBuilder;
        private bool _gpuChecked;
        private bool _gpuSkinUploaded;

        private bool NativeAvailable
        {
            get
            {
                if (!_nativeChecked)
                {
                    _nativeChecked = true;
                    _nativeAvailable = NativeBridge.IsAvailable;
                }
                return _nativeAvailable;
            }
        }

        // ── Public API ─────────────────────────────────────────────────
        public bool IsReady     { get; private set; }
        public int  VoxelCount  => _totalVoxels;
        public int  VertexCount => _vertCount;

        // ── GPU cloth solver reads these to sample SDF on GPU ──────────
        public float SDFOriginX   => _origin.x;
        public float SDFOriginY   => _origin.y;
        public float SDFOriginZ   => _origin.z;
        public int   SDFResX      => _resX;
        public int   SDFResY      => _resY;
        public int   SDFResZ      => _resZ;
        public float SDFCellSize  => _cellSizeActual;
        public float SDFMaxDist   => _maxDist;
        public float[] GetSDFDataInternal() => _sdfData;

        /// <summary>
        /// Returns the GPU SDF buffer if the GPU path is active, otherwise null.
        /// GPUClothSolver reads this for zero-copy SDF collision.
        /// </summary>
        public ComputeBuffer GPUSDFBuffer =>
            (_gpuBuilder != null && _gpuBuilder.IsReady) ? _gpuBuilder.FinalSDFBuffer : null;

        // ================================================================ //
        //  Initialisation
        // ================================================================ //

        /// <summary>
        /// Initialises the SDF collider from <paramref name="smr"/>.
        /// Pass <paramref name="boneWhitelist"/> to restrict to a body region.
        /// Vertices are decimated for performance; bone-filtered first so small
        /// regions (breasts, belly) retain sufficient density.
        /// </summary>
        public void Initialize(SkinnedMeshRenderer smr, HashSet<string> boneWhitelist = null)
        {
            IsReady = false;
            if (smr == null || smr.sharedMesh == null) return;

            Mesh         mesh    = smr.sharedMesh;
            Vector3[]    verts   = mesh.vertices;
            Vector3[]    normals = mesh.normals;
            BoneWeight[] weights = mesh.boneWeights;
            Transform[]  smrBones = smr.bones;

            _bones     = smrBones;
            _bindPoses = mesh.bindposes;
            _boneMatrices = new float4x4[_bones.Length];

            // ── 1. Filter by dominant bone ──
            var candidates = new List<int>(verts.Length);
            for (int i = 0; i < verts.Length; i++)
            {
                if (boneWhitelist != null)
                {
                    BoneWeight bw = weights[i];
                    int   dom = bw.boneIndex0; float dw = bw.weight0;
                    if (bw.weight1 > dw) { dw = bw.weight1; dom = bw.boneIndex1; }
                    if (bw.weight2 > dw) { dw = bw.weight2; dom = bw.boneIndex2; }
                    if (bw.weight3 > dw) {                   dom = bw.boneIndex3; }

                    if (dom < 0 || dom >= smrBones.Length || smrBones[dom] == null ||
                        !boneWhitelist.Contains(smrBones[dom].name))
                        continue;
                }
                candidates.Add(i);
            }

            // ── 2. Decimate after filtering ──
            int stride = Math.Max(1, decimationFactor);
            var included = new List<int>(candidates.Count / stride + 1);
            for (int i = 0; i < candidates.Count; i += stride)
                included.Add(candidates[i]);
            if (included.Count == 0 && candidates.Count > 0)
                included.Add(candidates[0]);

            // ── 3. Compact into arrays ──
            _vertCount     = included.Count;
            _bindLocal     = new float3[_vertCount];
            _bindNormal    = new float3[_vertCount];
            _boneIdx       = new int4[_vertCount];
            _boneW         = new float4[_vertCount];
            _skinnedPos    = new Vector3[_vertCount];
            _skinnedNormal = new float3[_vertCount];

            bool hasNormals = normals != null && normals.Length == verts.Length;
            for (int j = 0; j < _vertCount; j++)
            {
                int        src = included[j];
                Vector3    v   = verts[src];
                BoneWeight bw  = weights[src];

                _bindLocal[j] = new float3(v.x, v.y, v.z);
                _boneIdx[j]   = new int4(bw.boneIndex0, bw.boneIndex1, bw.boneIndex2, bw.boneIndex3);
                _boneW[j]     = new float4(bw.weight0,  bw.weight1,  bw.weight2,  bw.weight3);

                if (hasNormals)
                {
                    Vector3 n = normals[src];
                    _bindNormal[j] = new float3(n.x, n.y, n.z);
                }
                else
                {
                    _bindNormal[j] = new float3(0, 1, 0);
                }
            }

            // ── 4. Pre-allocate SDF at max size ──
            int maxVox = resolution * resolution * resolution;
            _sdfData     = new float[maxVox];
            _totalVoxels = 0;
            _frameCounter = 0;

            // ── 5. Pre-allocate native flat arrays ──
            if (NativeAvailable)
            {
                _nBindPos     = new float[_vertCount * 3];
                _nBindNormal  = new float[_vertCount * 3];
                _nBoneIdx     = new int[_vertCount * 4];
                _nBoneW       = new float[_vertCount * 4];
                _nSkinnedPos  = new float[_vertCount * 3];
                _nSkinnedNormal = new float[_vertCount * 3];
                for (int j = 0; j < _vertCount; j++)
                {
                    _nBindPos[j * 3]     = _bindLocal[j].x;
                    _nBindPos[j * 3 + 1] = _bindLocal[j].y;
                    _nBindPos[j * 3 + 2] = _bindLocal[j].z;
                    _nBindNormal[j * 3]     = _bindNormal[j].x;
                    _nBindNormal[j * 3 + 1] = _bindNormal[j].y;
                    _nBindNormal[j * 3 + 2] = _bindNormal[j].z;
                    _nBoneIdx[j * 4]     = _boneIdx[j].x;
                    _nBoneIdx[j * 4 + 1] = _boneIdx[j].y;
                    _nBoneIdx[j * 4 + 2] = _boneIdx[j].z;
                    _nBoneIdx[j * 4 + 3] = _boneIdx[j].w;
                    _nBoneW[j * 4]       = _boneW[j].x;
                    _nBoneW[j * 4 + 1]   = _boneW[j].y;
                    _nBoneW[j * 4 + 2]   = _boneW[j].z;
                    _nBoneW[j * 4 + 3]   = _boneW[j].w;
                }
                _nativeSDFHandle = NativeBridge.SDF_Create(maxVox, _vertCount);
            }

            // ── 6. Try GPU compute path ──
            if (!_gpuChecked)
            {
                _gpuChecked = true;
                try { _gpuBuilder = GPUSDFBuilder.TryCreate(); }
                catch { _gpuBuilder = null; }
            }
            if (_gpuBuilder != null)
            {
                _gpuBuilder.UploadBindData(_bindLocal, _bindNormal, _boneIdx, _boneW, _vertCount);
                _gpuSkinUploaded = false;
            }
        }

        // ================================================================ //
        //  Per-frame SDF update
        // ================================================================ //

        /// <summary>
        /// Call once per frame on the main thread (before cloth simulation).
        /// Reads bone transforms, skins decimated vertices, and rebuilds the SDF.
        /// </summary>
        public void UpdateSDF()
        {
            if (_vertCount == 0) return;

            _frameCounter++;
            if (_frameCounter % Math.Max(1, updateInterval) != 0 && IsReady)
                return;

            // ── 1. Build skinning matrices (main thread — Transform reads) ──
            for (int i = 0; i < _bones.Length; i++)
            {
                if (_bones[i] == null) continue;
                Matrix4x4 u = _bones[i].localToWorldMatrix * _bindPoses[i];
                _boneMatrices[i] = new float4x4(
                    u.m00, u.m01, u.m02, u.m03,
                    u.m10, u.m11, u.m12, u.m13,
                    u.m20, u.m21, u.m22, u.m23,
                    u.m30, u.m31, u.m32, u.m33);
            }

            // ── GPU fast path: skin + build SDF entirely on GPU ──
            if (_gpuBuilder != null && _gpuBuilder.IsReady)
            {
                // Upload bone matrices → dispatch GPU skinning → build SDF → readback
                _gpuBuilder.UploadBoneMatrices(_boneMatrices, _boneMatrices.Length);
                _gpuBuilder.DispatchSkinVertices(_vertCount);

                // Readback skinned positions for AABB computation (CPU-side)
                _gpuBuilder.ReadbackSkinnedVertices(_skinnedPos, _skinnedNormal, _vertCount);

                // AABB + grid dimensions (same as other paths)
                ComputeGridParams();

                // GPU SDF build + readback to _sdfData
                _gpuBuilder.BuildSDF(_vertCount, _origin, _resX, _resY, _resZ,
                                     _cellSizeActual, _maxDist, _sdfData);
                IsReady = true;
                return;
            }

            // ── 2. Skin vertices + normals (CPU: native or managed) ──
            if (NativeAvailable)
            {
                if (_nBoneMatsFlat == null || _nBoneMatsFlat.Length < _bones.Length * 16)
                    _nBoneMatsFlat = new float[_bones.Length * 16];
                for (int i = 0; i < _bones.Length; i++)
                {
                    if (_bones[i] == null) continue;
                    Matrix4x4 u = _bones[i].localToWorldMatrix * _bindPoses[i];
                    int off = i * 16;
                    _nBoneMatsFlat[off]      = u.m00; _nBoneMatsFlat[off + 1]  = u.m10; _nBoneMatsFlat[off + 2]  = u.m20; _nBoneMatsFlat[off + 3]  = u.m30;
                    _nBoneMatsFlat[off + 4]  = u.m01; _nBoneMatsFlat[off + 5]  = u.m11; _nBoneMatsFlat[off + 6]  = u.m21; _nBoneMatsFlat[off + 7]  = u.m31;
                    _nBoneMatsFlat[off + 8]  = u.m02; _nBoneMatsFlat[off + 9]  = u.m12; _nBoneMatsFlat[off + 10] = u.m22; _nBoneMatsFlat[off + 11] = u.m32;
                    _nBoneMatsFlat[off + 12] = u.m03; _nBoneMatsFlat[off + 13] = u.m13; _nBoneMatsFlat[off + 14] = u.m23; _nBoneMatsFlat[off + 15] = u.m33;
                }

                NativeBridge.SDF_SkinVertices(
                    _nBindPos, _nBindNormal, _nBoneIdx, _nBoneW,
                    _nBoneMatsFlat, _vertCount,
                    _nSkinnedPos, _nSkinnedNormal);

                for (int i = 0; i < _vertCount; i++)
                {
                    _skinnedPos[i] = new Vector3(_nSkinnedPos[i * 3], _nSkinnedPos[i * 3 + 1], _nSkinnedPos[i * 3 + 2]);
                    _skinnedNormal[i] = new float3(_nSkinnedNormal[i * 3], _nSkinnedNormal[i * 3 + 1], _nSkinnedNormal[i * 3 + 2]);
                }
            }

            // ── 3. AABB + grid dimensions ──
            ComputeGridParams();

            // ── 4. Build SDF (Native path) ──
            if (NativeAvailable && _nativeSDFHandle != 0)
            {
                float hashCell = math.max(0.04f, _cellSizeActual * 3f);
                float queryRadius = hashCell * 2f;
                NativeBridge.SDF_Build(
                    _nativeSDFHandle,
                    _nSkinnedPos, _nSkinnedNormal, _vertCount,
                    _origin.x, _origin.y, _origin.z,
                    _resX, _resY, _resZ,
                    _cellSizeActual, _maxDist,
                    hashCell, queryRadius,
                    _sdfData);
            }

            IsReady = true;
        }

        /// <summary>
        /// Compute AABB from skinned positions and derive grid dimensions.
        /// Shared by GPU, native, and managed paths.
        /// </summary>
        private void ComputeGridParams()
        {
            float3 aabbMin = new float3(_skinnedPos[0].x, _skinnedPos[0].y, _skinnedPos[0].z);
            float3 aabbMax = aabbMin;
            for (int i = 1; i < _vertCount; i++)
            {
                Vector3 sp = _skinnedPos[i];
                float3 f = new float3(sp.x, sp.y, sp.z);
                aabbMin = math.min(aabbMin, f);
                aabbMax = math.max(aabbMax, f);
            }
            float totalPad = padding + surfaceOffset;
            aabbMin -= totalPad;
            aabbMax += totalPad;

            float3 extent = aabbMax - aabbMin;
            int res = Math.Max(8, Math.Min(resolution, 48));
            float cs = math.max(extent.x, math.max(extent.y, extent.z)) / (res - 1);
            cs = math.max(cs, 0.005f);

            _resX = Math.Max(2, Math.Min(res, (int)math.ceil(extent.x / cs) + 1));
            _resY = Math.Max(2, Math.Min(res, (int)math.ceil(extent.y / cs) + 1));
            _resZ = Math.Max(2, Math.Min(res, (int)math.ceil(extent.z / cs) + 1));
            _totalVoxels    = _resX * _resY * _resZ;
            _origin         = aabbMin;
            _cellSizeActual = cs;
            _invCellSize    = 1f / cs;
            _maxDist        = cs * math.max(_resX, math.max(_resY, _resZ)) * 2f;

            if (_sdfData.Length < _totalVoxels)
                _sdfData = new float[_totalVoxels];
        }


        /// <summary>
        /// Release GPU and native resources. Call when this collider is no longer needed.
        /// </summary>
        public void Release()
        {
            _gpuBuilder?.Dispose();
            _gpuBuilder = null;

            if (NativeAvailable && _nativeSDFHandle != 0)
            {
                NativeBridge.SDF_Destroy(_nativeSDFHandle);
                _nativeSDFHandle = 0;
            }
        }
    }
}
