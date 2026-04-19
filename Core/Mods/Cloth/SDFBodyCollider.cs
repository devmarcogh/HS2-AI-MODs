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

        // ── Triangle topology (for triangle-based SDF) ──────────────────
        private int[]       _triangleIndices;   // remapped triangle indices into decimated vertex set
        private int[]       _nTriangleIndices;  // flat copy for native call
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

        // ── Spatial hash for fast nearest-vertex during SDF build ──────
        private readonly SpatialHashGrid _vertexGrid = new SpatialHashGrid();
        private readonly ThreadLocal<List<int>> _tlQueryBuf =
            new ThreadLocal<List<int>>(() => new List<int>(32));

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
        public float SDFInvCellSize => _invCellSize;
        public float[] SDFData    => _sdfData;

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

            // ── 4. Build remapped triangle indices ──
            // Map original mesh vertex indices → decimated index set
            var originalToDecimated = new Dictionary<int, int>(_vertCount);
            for (int j = 0; j < _vertCount; j++)
                originalToDecimated[included[j]] = j;

            int[] origTris = mesh.triangles;
            var remappedTris = new List<int>(origTris.Length);
            for (int t = 0; t < origTris.Length; t += 3)
            {
                int a, b, c;
                if (originalToDecimated.TryGetValue(origTris[t], out a) &&
                    originalToDecimated.TryGetValue(origTris[t + 1], out b) &&
                    originalToDecimated.TryGetValue(origTris[t + 2], out c))
                {
                    remappedTris.Add(a);
                    remappedTris.Add(b);
                    remappedTris.Add(c);
                }
            }
            _triangleIndices = remappedTris.ToArray();

            // ── 5. Pre-allocate SDF at max size ──
            int maxVox = resolution * resolution * resolution;
            _sdfData     = new float[maxVox];
            _totalVoxels = 0;
            _frameCounter = 0;

            // ── 6. Pre-allocate native flat arrays ──
            if (NativeAvailable)
            {
                _nBindPos     = new float[_vertCount * 3];
                _nBindNormal  = new float[_vertCount * 3];
                _nBoneIdx     = new int[_vertCount * 4];
                _nBoneW       = new float[_vertCount * 4];
                _nSkinnedPos  = new float[_vertCount * 3];
                _nSkinnedNormal = new float[_vertCount * 3];
                _nTriangleIndices = _triangleIndices; // already int[], share reference
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
            else
            {
                SkinAll();
            }

            // ── 3. AABB + grid dimensions ──
            ComputeGridParams();

            // ── 4. Build SDF (native or managed) ──
            float hashCell = math.max(0.04f, _cellSizeActual * 3f);
            float queryRadius = hashCell * 2f;

            if (NativeAvailable && _nativeSDFHandle != 0)
            {
                NativeBridge.SDF_Build(
                    _nativeSDFHandle,
                    _nSkinnedPos, _nSkinnedNormal, _vertCount,
                    _nTriangleIndices, _nTriangleIndices != null ? _nTriangleIndices.Length : 0,
                    _origin.x, _origin.y, _origin.z,
                    _resX, _resY, _resZ,
                    _cellSizeActual, _maxDist,
                    hashCell, queryRadius,
                    _sdfData);
            }
            else
            {
                _vertexGrid.Build(_skinnedPos, hashCell);
                BuildSDFParallel(queryRadius);
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

        // ================================================================ //
        //  SDF Queries (thread-safe, read-only after UpdateSDF)
        // ================================================================ //

        /// <summary>
        /// Samples the signed distance at a world-space position via trilinear interpolation.
        /// Positive = outside body, negative = inside body.
        /// Returns _maxDist if outside the SDF volume.
        /// </summary>
        public float SampleDistance(float3 worldPos)
        {
            float3 local = (worldPos - _origin) * _invCellSize;

            if (local.x < 0 || local.y < 0 || local.z < 0 ||
                local.x >= _resX - 1 || local.y >= _resY - 1 || local.z >= _resZ - 1)
                return _maxDist;

            int ix = (int)local.x, iy = (int)local.y, iz = (int)local.z;
            float fx = local.x - ix, fy = local.y - iy, fz = local.z - iz;

            float c000 = _sdfData[Idx(ix,   iy,   iz  )];
            float c100 = _sdfData[Idx(ix+1, iy,   iz  )];
            float c010 = _sdfData[Idx(ix,   iy+1, iz  )];
            float c110 = _sdfData[Idx(ix+1, iy+1, iz  )];
            float c001 = _sdfData[Idx(ix,   iy,   iz+1)];
            float c101 = _sdfData[Idx(ix+1, iy,   iz+1)];
            float c011 = _sdfData[Idx(ix,   iy+1, iz+1)];
            float c111 = _sdfData[Idx(ix+1, iy+1, iz+1)];

            float c00 = c000 + (c100 - c000) * fx;
            float c10 = c010 + (c110 - c010) * fx;
            float c01 = c001 + (c101 - c001) * fx;
            float c11 = c011 + (c111 - c011) * fx;

            float c0 = c00 + (c10 - c00) * fy;
            float c1 = c01 + (c11 - c01) * fy;

            return c0 + (c1 - c0) * fz;
        }

        /// <summary>
        /// Samples the SDF gradient (outward surface normal direction) via central differences.
        /// </summary>
        public float3 SampleGradient(float3 worldPos)
        {
            // Clamp factor to avoid unstable tiny steps or over-smoothed normals.
            float factor = Mathf.Clamp(gradientSampleFactor, 0.75f, 2.0f);
            float h = _cellSizeActual * factor;

            float dx = SampleDistance(worldPos + new float3(h, 0, 0)) -
                       SampleDistance(worldPos - new float3(h, 0, 0));
            float dy = SampleDistance(worldPos + new float3(0, h, 0)) -
                       SampleDistance(worldPos - new float3(0, h, 0));
            float dz = SampleDistance(worldPos + new float3(0, 0, h)) -
                       SampleDistance(worldPos - new float3(0, 0, h));

            return new float3(dx, dy, dz) / (2f * h);
        }

        // ================================================================ //
        //  Internal — LBS skinning + normal transform
        // ================================================================ //

        private void SkinAll()
        {
            float3[]   bind    = _bindLocal;
            float3[]   bindN   = _bindNormal;
            int4[]     bIdx    = _boneIdx;
            float4[]   bW      = _boneW;
            float4x4[] bMats   = _boneMatrices;
            Vector3[]  outPos  = _skinnedPos;
            float3[]   outNorm = _skinnedNormal;
            int        count   = _vertCount;

            if (count > 64)
            {
                Parallel.For(0, count, i =>
                {
                    SkinVertex(bind[i], bindN[i], bIdx[i], bW[i], bMats,
                               out outPos[i], out outNorm[i]);
                });
            }
            else
            {
                for (int i = 0; i < count; i++)
                    SkinVertex(bind[i], bindN[i], bIdx[i], bW[i], bMats,
                               out outPos[i], out outNorm[i]);
            }
        }

        private static void SkinVertex(float3 bindP, float3 bindN, int4 idx, float4 w,
                                        float4x4[] mats, out Vector3 outPos, out float3 outNorm)
        {
            float4 p4 = new float4(bindP, 1f);
            float4 n4 = new float4(bindN, 0f); // w=0: direction, no translation

            float3 pos = float3.zero;
            float3 nor = float3.zero;

            if (w.x > 0f) { pos += math.mul(mats[idx.x], p4).xyz * w.x;
                             nor += math.mul(mats[idx.x], n4).xyz * w.x; }
            if (w.y > 0f) { pos += math.mul(mats[idx.y], p4).xyz * w.y;
                             nor += math.mul(mats[idx.y], n4).xyz * w.y; }
            if (w.z > 0f) { pos += math.mul(mats[idx.z], p4).xyz * w.z;
                             nor += math.mul(mats[idx.z], n4).xyz * w.z; }
            if (w.w > 0f) { pos += math.mul(mats[idx.w], p4).xyz * w.w;
                             nor += math.mul(mats[idx.w], n4).xyz * w.w; }

            outPos  = new Vector3(pos.x, pos.y, pos.z);
            outNorm = math.normalizesafe(nor, new float3(0, 1, 0));
        }

        // ================================================================ //
        //  Internal — Parallel SDF build (vertex-distance)
        // ================================================================ //

        /// <summary>
        /// For each voxel, finds the nearest skinned vertex via the spatial hash
        /// and writes the signed distance. Fully parallel — no write contention
        /// because each voxel writes to its own unique index.
        /// </summary>
        private void BuildSDFParallel(float queryRadius)
        {
            float[]    sdf    = _sdfData;
            int        rx     = _resX, ry = _resY, rz = _resZ;
            int        total  = _totalVoxels;
            float3     origin = _origin;
            float      cs     = _cellSizeActual;
            float      maxD   = _maxDist;
            Vector3[]  vPos   = _skinnedPos;
            float3[]   vNorm  = _skinnedNormal;
            float      qr     = queryRadius;
            SpatialHashGrid grid = _vertexGrid;

            if (total > 256)
            {
                Parallel.For(0, total, vi =>
                {
                    sdf[vi] = ComputeVoxelSDF(vi, rx, ry, origin, cs, maxD,
                                              vPos, vNorm, qr, grid, _tlQueryBuf.Value);
                });
            }
            else
            {
                var buf = new List<int>(32);
                for (int vi = 0; vi < total; vi++)
                    sdf[vi] = ComputeVoxelSDF(vi, rx, ry, origin, cs, maxD,
                                              vPos, vNorm, qr, grid, buf);
            }

            // 3×3×3 Gaussian smoothing pass (matches GPU + native paths)
            SmoothSDFInPlace(sdf, rx, ry, rz, total);
        }

        /// <summary>
        /// In-place 3×3×3 weighted blur over the SDF volume.
        /// Smooths Voronoi valleys so the gradient is continuous for XPBD.
        /// </summary>
        private static void SmoothSDFInPlace(float[] sdf, int rx, int ry, int rz, int total)
        {
            float[] temp = new float[total];
            int rxy = rx * ry;

            for (int vi = 0; vi < total; vi++)
            {
                int vz = vi / rxy;
                int rem = vi - vz * rxy;
                int vy2 = rem / rx;
                int vx2 = rem - vy2 * rx;

                float sum = 0f;
                float wTotal = 0f;

                for (int dz = -1; dz <= 1; dz++)
                for (int dy = -1; dy <= 1; dy++)
                for (int dx = -1; dx <= 1; dx++)
                {
                    int nx = vx2 + dx, ny = vy2 + dy, nz = vz + dz;
                    if (nx < 0 || nx >= rx || ny < 0 || ny >= ry || nz < 0 || nz >= rz)
                        continue;
                    int manhattan = Math.Abs(dx) + Math.Abs(dy) + Math.Abs(dz);
                    float w;
                    if      (manhattan == 0) w = 8f;
                    else if (manhattan == 1) w = 4f;
                    else if (manhattan == 2) w = 2f;
                    else                     w = 1f;
                    sum += sdf[nz * rxy + ny * rx + nx] * w;
                    wTotal += w;
                }
                temp[vi] = sum / wTotal;
            }
            Array.Copy(temp, sdf, total);
        }

        /// <summary>
        /// Pure function: compute signed distance for a single voxel.
        /// Thread-safe — only reads from shared data, writes to caller-provided buffer.
        /// </summary>
        private static float ComputeVoxelSDF(
            int vi, int rx, int ry,
            float3 origin, float cs, float maxD,
            Vector3[] vPos, float3[] vNorm,
            float qr, SpatialHashGrid grid, List<int> queryBuf)
        {
            // Voxel index → 3D coordinates
            int rxy    = rx * ry;
            int vz     = vi / rxy;
            int remain = vi - vz * rxy;
            int vy     = remain / rx;
            int vx     = remain - vy * rx;

            Vector3 voxelPos = new Vector3(
                origin.x + vx * cs,
                origin.y + vy * cs,
                origin.z + vz * cs);

            // Query spatial hash for nearby vertices
            grid.Query(voxelPos, qr, queryBuf);

            // K-nearest for sign voting (matches GPU + native paths)
            const int K_NEAREST = 4;
            float[] kDistSq = new float[K_NEAREST];
            int[]   kIdx    = new int[K_NEAREST];
            for (int k = 0; k < K_NEAREST; k++) { kDistSq[k] = float.MaxValue; kIdx[k] = -1; }

            for (int qi = 0; qi < queryBuf.Count; qi++)
            {
                int pidx = queryBuf[qi];
                float dx = voxelPos.x - vPos[pidx].x;
                float dy = voxelPos.y - vPos[pidx].y;
                float dz = voxelPos.z - vPos[pidx].z;
                float dSq = dx * dx + dy * dy + dz * dz;

                if (dSq < kDistSq[K_NEAREST - 1])
                {
                    kDistSq[K_NEAREST - 1] = dSq;
                    kIdx[K_NEAREST - 1] = pidx;
                    // Bubble sort towards front
                    for (int s = K_NEAREST - 1; s > 0; s--)
                    {
                        if (kDistSq[s] < kDistSq[s - 1])
                        {
                            float tmpD = kDistSq[s]; kDistSq[s] = kDistSq[s-1]; kDistSq[s-1] = tmpD;
                            int   tmpI = kIdx[s];    kIdx[s]    = kIdx[s-1];    kIdx[s-1]    = tmpI;
                        }
                    }
                }
            }

            if (kIdx[0] < 0)
                return maxD;

            float dist = (float)Math.Sqrt(kDistSq[0]);

            // Distance-weighted sign voting from K nearest vertices
            float signAccum = 0f;
            float epsilon = kDistSq[0] * 0.01f + 1e-10f;
            for (int n = 0; n < K_NEAREST; n++)
            {
                if (kIdx[n] < 0) break;
                float3 toVoxel = new float3(
                    voxelPos.x - vPos[kIdx[n]].x,
                    voxelPos.y - vPos[kIdx[n]].y,
                    voxelPos.z - vPos[kIdx[n]].z);
                float vote   = math.dot(toVoxel, vNorm[kIdx[n]]);
                float weight = 1f / (kDistSq[n] + epsilon);
                signAccum += vote * weight;
            }

            return (signAccum >= 0f) ? dist : -dist;
        }

        // ── Voxel indexing ─────────────────────────────────────────────
        private int Idx(int x, int y, int z) => x + y * _resX + z * _resX * _resY;

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
