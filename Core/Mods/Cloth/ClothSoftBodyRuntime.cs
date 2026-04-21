using AIChara;
using BepInEx.Configuration;
using System;
using System.Collections.Generic;
using System.Reflection;
using System.Threading.Tasks;
using UnityEngine;

namespace StudioModsMSG
{
    /// <summary>
    /// xPBD cloth physics runtime (Extended Position-Based Dynamics, Macklin et al. 2016).
    ///
    /// Per substep:
    ///   1. Skeleton follow — advect free verts by the same world-space delta as the
    ///      pinned vertices (or by character root movement when no pins exist).
    ///   2. Integrate gravity into velocity; predict: p = x + v·dt.
    ///   3. Iterate xPBD constraint projections (edge stretch, pin hard-constraints,
    ///      mirrored DynamicBone collider collision).
    ///   4. Derive velocity from position change, apply damping, commit positions.
    /// </summary>
    [DefaultExecutionOrder(99000)] // Run after IK/FK mods so bone transforms are final
    class ClothSoftBodyRuntime : MonoBehaviour
    {

        
        private struct MirrorCollider
        {
            public Vector3 Center;
            public float Radius;
            public bool IsCapsule;
            
            // Pre-calculated Capsule Segment Math
            public Vector3 P1;
            public Vector3 V;
            public float VDotV;

            // Magnetic interaction: 0=Repel (normal), 1=Attract, 2=Off
            public byte  MagneticMode;
            public float MagneticStrength;
            public float MagneticRange;
        }

        // ── Optimisation: MirrorCollider flat array (avoid List enumerator in hot loop) ──

        private MirrorCollider[] _mirrorColArr = new MirrorCollider[64];
        private int              _mirrorColCount;

        // ── Optimisation: reflection cache for DynamicBoneCollider fields ──────────
        private static FieldInfo _cachedFieldRadius;
        private static FieldInfo _cachedFieldHeight;
        private static bool      _reflectionCached;

        // ── Optimisation: DynamicBoneCollider component cache ─────────────────────
        private DynamicBoneColliderBase[] _dbColliderCache;
        private int                       _dbColliderCacheFrame = -1;
        private const int                 DbColliderCacheInterval = 30; // re-query every N frames

        // ── Optimisation: MeshCollider throttle (PhysX mesh cook is very expensive) ──
        private int _meshColliderFrame;
        private const int MeshColliderUpdateInterval = 10;

        // ── Optimisation: per-frame LBS skinned positions for pinned/blend vertices ──
        // NOTE: these are now stored PER MESH in ClothMeshState.SkinnedPositions / PrevSkinnedPositions
        // to avoid cross-mesh corruption when multiple meshes are active simultaneously.
        private Matrix4x4[] _skinMats;


        // ── Optimisation: thread-local query buffer for parallel collision ───

        // ── Native acceleration (flat arrays for P/Invoke) ──────────────────────
        private bool   _nativeChecked;
        private bool   _nativeAvailable;
        private long   _nativeClothHandle;
        private float[] _nPos, _nVel, _nPred, _nInvMass;
        private int[]   _nIsPinned;
        private float[] _nSkinMatsFlat;
        private NativeBridge.NativeCollider[] _nColliders;

        // ── GPU acceleration (compute shader cloth solver) ──────────────
        private GPUClothSolver _gpuClothSolver;
        private bool           _gpuClothChecked;

        private bool GPUClothAvailable
        {
            get
            {
                if (!_gpuClothChecked)
                {
                    _gpuClothChecked = true;
                    try { _gpuClothSolver = GPUClothSolver.TryCreate(); }
                    catch { _gpuClothSolver = null; }
                }
                return _gpuClothSolver != null && _gpuClothSolver.IsReady;
            }
        }

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

        private void EnsureNativeBuffers(ClothMeshState s)
        {
            int n = s.VertCount;
            if (_nPos == null || _nPos.Length < n * 3)
            {
                _nPos     = new float[n * 3];
                _nVel     = new float[n * 3];
                _nPred    = new float[n * 3];
                _nInvMass = new float[n];
                _nIsPinned = new int[n];
            }
        }

        private void SyncToNative(ClothMeshState s)
        {
            int n = s.VertCount;
            for (int i = 0; i < n; i++)
            {
                int i3 = i * 3;
                _nPos[i3]     = s.Position[i].x;     _nPos[i3 + 1]     = s.Position[i].y;     _nPos[i3 + 2]     = s.Position[i].z;
                _nVel[i3]     = s.Velocity[i].x;     _nVel[i3 + 1]     = s.Velocity[i].y;     _nVel[i3 + 2]     = s.Velocity[i].z;
                _nInvMass[i]  = s.InvMass[i];
                _nIsPinned[i] = s.IsPinned[i] ? 1 : 0;
            }
        }

        private void SyncFromNative(ClothMeshState s)
        {
            // Only read back FREE particles. Pinned and dummy-padding particles
            // [FreeVertCount..VertCount-1] must NEVER be overwritten by the solver.
            int freeN = s.FreeVertCount;
            for (int i = 0; i < freeN; i++)
            {
                int i3 = i * 3;
                s.Position[i]     = new Vector3(_nPos[i3], _nPos[i3 + 1], _nPos[i3 + 2]);
                s.Velocity[i]     = new Vector3(_nVel[i3], _nVel[i3 + 1], _nVel[i3 + 2]);
                s.PredPosition[i] = new Vector3(_nPred[i3], _nPred[i3 + 1], _nPred[i3 + 2]);
            }
        }

        private void BuildNativeColliders()
        {
            if (_nColliders == null || _nColliders.Length < _mirrorColCount)
                _nColliders = new NativeBridge.NativeCollider[Mathf.NextPowerOfTwo(Mathf.Max(8, _mirrorColCount))];
            for (int c = 0; c < _mirrorColCount; c++)
            {
                MirrorCollider mc = _mirrorColArr[c];
                _nColliders[c] = new NativeBridge.NativeCollider
                {
                    cx = mc.Center.x, cy = mc.Center.y, cz = mc.Center.z,
                    radius = mc.Radius,
                    isCapsule = mc.IsCapsule ? 1 : 0,
                    p1x = mc.P1.x, p1y = mc.P1.y, p1z = mc.P1.z,
                    vx = mc.V.x, vy = mc.V.y, vz = mc.V.z,
                    vDotV = mc.VDotV,
                    magneticMode = mc.MagneticMode,
                    magneticStrength = mc.MagneticStrength,
                    magneticRange = mc.MagneticRange,
                };
            }
        }

        // ── SDF collider ────────────────────────────────────────────────────────
        private SDFBodyCollider _sdfProxy;

        public int  SDFVoxelCount      { get; private set; }
        public int  SDFVertexCount     { get; private set; }

        /// <summary>
        /// Toggle mirrored DynamicBone colliders on/off independently.
        /// </summary>
        public bool UseMirrorColliders { get; set; } = true;

        /// <summary>
        /// Toggle SDF body collision on/off independently.
        /// The SDF can be built and kept ready even when disabled here.
        /// </summary>
        public bool UseSDFColliders { get; set; } = false;

        /// <summary>
        /// When true, the simulation is frozen — cloth stays in its current shape.
        /// </summary>
        public bool SimulationPaused { get; set; } = false;

        /// <summary>
        /// True when any mesh is being manually deformed (disables camera input).
        /// </summary>
        public static bool ManualDeformActive { get; set; } = false;

        private void EnsureMirrorCapacity(int needed)
        {
            if (_mirrorColArr.Length < needed)
                _mirrorColArr = new MirrorCollider[Mathf.NextPowerOfTwo(needed)];
        }

        private void UpdateMirrorColliders()
        {
            _mirrorColCount = 0;
            if (chaCtrl == null) return;

            // ── Optional mirrored DynamicBone colliders ──────────
            if (UseMirrorColliders)
            {
                // Cache GetComponentsInChildren — re-query only every N frames
                int frame = Time.frameCount;
                if (_dbColliderCache == null || (frame - _dbColliderCacheFrame) >= DbColliderCacheInterval)
                {
                    _dbColliderCache = chaCtrl.GetComponentsInChildren<DynamicBoneColliderBase>(true);
                    _dbColliderCacheFrame = frame;
                }

                // Cache reflection FieldInfo lookups (one-time cost)
                if (!_reflectionCached)
                {
                    var sampleType = typeof(DynamicBoneCollider);
                    _cachedFieldRadius = sampleType.GetField("m_Radius");
                    _cachedFieldHeight = sampleType.GetField("m_Height");
                    _reflectionCached  = true;
                }

                int dbCount = _dbColliderCache != null ? _dbColliderCache.Length : 0;
                EnsureMirrorCapacity(dbCount);

                for (int ci = 0; ci < dbCount; ci++)
                {
                    var dbCol = _dbColliderCache[ci];
                    if (dbCol == null || !dbCol.enabled) continue;

                    float radius = 0;
                    float height = 0;

                    if (_cachedFieldRadius != null) radius = (float)_cachedFieldRadius.GetValue(dbCol);
                    if (_cachedFieldHeight != null) height = (float)_cachedFieldHeight.GetValue(dbCol);

                    MirrorCollider mc = new MirrorCollider();
                    mc.Center    = dbCol.transform.TransformPoint(dbCol.m_Center);
                    mc.Radius    = radius * Mathf.Abs(dbCol.transform.lossyScale.x);
                    mc.IsCapsule = height > 0;

                    if (mc.IsCapsule)
                    {
                        float actualHeight = height * Mathf.Abs(dbCol.transform.lossyScale.y);
                        Vector3 dir = Vector3.up;
                        if (dbCol.m_Direction == DynamicBoneColliderBase.Direction.X) dir = Vector3.right;
                        else if (dbCol.m_Direction == DynamicBoneColliderBase.Direction.Z) dir = Vector3.forward;

                        Vector3 worldDir = dbCol.transform.TransformDirection(dir);
                        float   halfH    = Mathf.Max(0, actualHeight * 0.5f - mc.Radius);

                        mc.P1    = mc.Center + worldDir * halfH;
                        Vector3 p2 = mc.Center - worldDir * halfH;
                        mc.V     = p2 - mc.P1;
                        mc.VDotV = Vector3.Dot(mc.V, mc.V);
                    }

                    _mirrorColArr[_mirrorColCount++] = mc;
                }
            }
            else
            {
                _mirrorColCount = 0;
            }

            // ── Custom cloth collider proxies (workspace folder items) ───
            var proxies = ClothColliderProxy.All;
            int proxyCount = proxies.Count;
            if (proxyCount > 0)
            {
                EnsureMirrorCapacity(_mirrorColCount + proxyCount);
                for (int pi = 0; pi < proxyCount; pi++)
                {
                    var proxy = proxies[pi];
                    if (proxy == null || !proxy.isActiveAndEnabled) continue;
                    // Off-mode colliders skip physics entirely
                    if (proxy.MagneticMode == ColliderMagneticMode.Off) continue;

                    proxy.GetWorldCapsule(out Vector3 center, out Vector3 worldDir,
                                          out float worldRadius, out float halfH);

                    MirrorCollider mc = new MirrorCollider();
                    mc.Center          = center;
                    mc.Radius          = worldRadius;
                    mc.IsCapsule       = halfH > 0.0001f;
                    mc.MagneticMode    = (byte)proxy.MagneticMode;
                    mc.MagneticStrength = proxy.MagneticStrength;
                    mc.MagneticRange   = proxy.MagneticRange;

                    if (mc.IsCapsule)
                    {
                        mc.P1    = center + worldDir * halfH;
                        Vector3 p2 = center - worldDir * halfH;
                        mc.V     = p2 - mc.P1;
                        mc.VDotV = Vector3.Dot(mc.V, mc.V);
                    }

                    _mirrorColArr[_mirrorColCount++] = mc;
                }
            }

            // ── Joan6694 scene colliders (Dynamic Bone Capsule Colliders added from QuickAccess) ──
            var joan = Joan6694ColliderWatcher.Colliders;
            int joanCount = joan.Count;
            if (joanCount > 0)
            {
                EnsureMirrorCapacity(_mirrorColCount + joanCount);
                for (int ji = 0; ji < joanCount; ji++)
                {
                    DynamicBoneColliderBase dbCol = joan[ji];
                    if (dbCol == null || !dbCol.enabled || !dbCol.gameObject.activeInHierarchy) continue;

                    float radius = 0f;
                    float height = 0f;
                    if (_cachedFieldRadius != null) radius = (float)_cachedFieldRadius.GetValue(dbCol);
                    if (_cachedFieldHeight != null) height = (float)_cachedFieldHeight.GetValue(dbCol);

                    MirrorCollider mc = new MirrorCollider();
                    mc.Center    = dbCol.transform.TransformPoint(dbCol.m_Center);
                    mc.Radius    = radius * Mathf.Abs(dbCol.transform.lossyScale.x);
                    mc.IsCapsule = height > 0;

                    if (mc.IsCapsule)
                    {
                        float actualHeight = height * Mathf.Abs(dbCol.transform.lossyScale.y);
                        Vector3 dir = Vector3.up;
                        if (dbCol.m_Direction == DynamicBoneColliderBase.Direction.X) dir = Vector3.right;
                        else if (dbCol.m_Direction == DynamicBoneColliderBase.Direction.Z) dir = Vector3.forward;

                        Vector3 worldDir2 = dbCol.transform.TransformDirection(dir);
                        float   halfH2    = Mathf.Max(0, actualHeight * 0.5f - mc.Radius);

                        mc.P1    = mc.Center + worldDir2 * halfH2;
                        Vector3 p2j = mc.Center - worldDir2 * halfH2;
                        mc.V     = p2j - mc.P1;
                        mc.VDotV = Vector3.Dot(mc.V, mc.V);
                    }

                    _mirrorColArr[_mirrorColCount++] = mc;
                }
            }
        }

        // ------------------------------------------------------------------ //
        // Cloth state list
        // ------------------------------------------------------------------ //
        private ChaControl chaCtrl;
        private readonly List<ClothPhysicsEntry> entries       = new List<ClothPhysicsEntry>();
        private readonly List<ClothMeshState>    activeStates  = new List<ClothMeshState>();

        // Female clothing slot names (matches CharaEditorController.FEMALE_CLOTHES_NAME)
        private static readonly string[] ClothSlotNames =
            { "Top", "Bot", "Inner_t", "Inner_b", "Gloves", "Panst", "Socks", "Shoes" };

        // ------------------------------------------------------------------ //
        // Public interface
        // ------------------------------------------------------------------ //
        public IReadOnlyList<ClothPhysicsEntry> Entries => entries;

        public void Attach(ChaControl control)
        {
            chaCtrl = control;
            BuildClothEntries();
        }

        public void ActivateMesh(ClothMeshState state)
        {
            if (state == null || state.IsActive) return;
            if (!SetupMeshState(state)) return;
            state.IsActive = true;
            if (!activeStates.Contains(state)) activeStates.Add(state);

            // Upload topology to GPU solver (edges/bends grouped by color)
            if (GPUClothAvailable)
                _gpuClothSolver.UploadTopology(state);

            WarmUpClothState(state);
        }

        public void DeactivateMesh(ClothMeshState state)
        {
            if (state == null || !state.IsActive) return;
            TeardownMeshState(state);
            state.IsActive = false;
            activeStates.Remove(state);
        }

        /// <summary>
        /// Initialises the cloth state for simulation.
        /// Primes the per-mesh LBS cache so frame-1 bone-follow delta is ~zero,
        /// and starts the warmup counter so initial collisions are heavily damped
        /// (prevents the "init pop" when all vertices suddenly react to colliders).
        /// </summary>
        private void WarmUpClothState(ClothMeshState state)
        {
            if (state == null) return;

            int n = state.VertCount;
            if (n <= 0) return;

            // Prime per-mesh skin caches from the current LBS pose.
            // This ensures that on frame 1, PrevSkinnedPositions ≈ SkinnedPositions[frame1],
            // so the bone-follow delta is near zero instead of a large teleport.
            if (state.SkinnedPositions == null || state.SkinnedPositions.Length < n)
                state.SkinnedPositions = new Vector3[n];
            if (state.PrevSkinnedPositions == null || state.PrevSkinnedPositions.Length < n)
                state.PrevSkinnedPositions = new Vector3[n];

            // Compute current LBS to prime the buffers
            if (state.SrcBindVerts != null && state.BindPoses != null &&
                state.SkinBones != null && state.SkinBones.Length > 0)
            {
                int bc = state.SkinBones.Length;
                if (_skinMats == null || _skinMats.Length < bc) _skinMats = new Matrix4x4[bc];
                for (int b = 0; b < bc; b++)
                    _skinMats[b] = (state.SkinBones[b] != null && b < state.BindPoses.Length)
                        ? state.SkinBones[b].localToWorldMatrix * state.BindPoses[b]
                        : Matrix4x4.identity;

                for (int i = 0; i < n; i++)
                {
                    Vector3   v  = state.SrcBindVerts[i];
                    BoneWeight bw = state.VertexBoneWeights[i];
                    state.SkinnedPositions[i] =
                        (Vector3)_skinMats[bw.boneIndex0].MultiplyPoint3x4(v) * bw.weight0 +
                        (Vector3)_skinMats[bw.boneIndex1].MultiplyPoint3x4(v) * bw.weight1 +
                        (Vector3)_skinMats[bw.boneIndex2].MultiplyPoint3x4(v) * bw.weight2 +
                        (Vector3)_skinMats[bw.boneIndex3].MultiplyPoint3x4(v) * bw.weight3;
                }
                Array.Copy(state.SkinnedPositions, state.PrevSkinnedPositions, n);
            }
            else
            {
                // Fallback: copy from current positions
                Array.Copy(state.Position, state.SkinnedPositions, n);
                Array.Copy(state.Position, state.PrevSkinnedPositions, n);
            }

            // Start warmup: first N frames use heavy damping to let the cloth settle.
            state.WarmupFramesRemaining = ClothMeshState.WarmupDuration;


            WriteMesh(state);
        }

        // ------------------------------------------------------------------ //
        // Triggered-simulation helpers
        // ------------------------------------------------------------------ //

        private void OnDisable()
        {
            // Deactivate all on component disable
            for (int i = activeStates.Count - 1; i >= 0; i--)
                DeactivateMesh(activeStates[i]);

            // Release GPU cloth solver
            _gpuClothSolver?.Dispose();
            _gpuClothSolver = null;
            _gpuClothChecked = false;
        }

        private void LateUpdate()
        {
            if (chaCtrl == null || activeStates.Count == 0) return;

            // Global pause: freeze all meshes.
            if (SimulationPaused) return;

            // --- CPU optimisation: check whether any mesh actually needs work ---
            // Skip expensive mirror-collider and SDF updates when every active mesh
            // is individually paused (avoids burning CPU for a frozen scene).
            bool anyRunning = false;
            for (int ai = 0; ai < activeStates.Count; ai++)
            {
                ClothMeshState st = activeStates[ai];
                if (st == null || st.Renderer == null) continue;
                if (!st.SimulationPaused) { anyRunning = true; break; }
            }
            if (!anyRunning) return;

            UpdateMirrorColliders();

            // Update SDF body collision field once per frame before running substeps
            if (UseSDFColliders && _sdfProxy != null)
            {
                _sdfProxy.UpdateSDF();
                SDFVoxelCount  = _sdfProxy.VoxelCount;
                SDFVertexCount = _sdfProxy.VertexCount;
            }

            // --- Manual Deformation: resolve cursor world position once per frame ---

            float dt = Mathf.Clamp(Time.deltaTime, 0.001f, 0.05f);

            foreach (ClothMeshState state in activeStates)
            {
                if (state == null || state.Renderer == null) continue;

                // Per-mesh pause: skip simulation but keep the mesh rendered as-is.
                if (state.SimulationPaused) continue;


                // LBS skinning + per-vertex bone-follow using per-mesh cached buffers.
                SkinAndFollowBones(state);

                // Substeps is a quality multiplier (0.25..1). Map to concrete count.
                int   substeps = Mathf.Max(1, Mathf.RoundToInt(state.Params.Substeps));
                float subDt    = dt / substeps;

                // 2-tier: GPU → Native (C++)
                if (GPUClothAvailable)
                {
                    SimulateAllSubstepsGPU(state, dt, substeps);
                }
                else if (NativeAvailable)
                {
                    for (int sub = 0; sub < substeps; sub++)
                        SimulateStepNative(state, subDt);
                }


                // Blend zone: lerp simulated positions toward LBS for partially-pinned vertices
                ApplyPinBlend(state);

                // FINAL HARD LOCK: no matter what any solver path did, pinned vertices
                // must always be at their exact LBS position before rendering.
                // Indices [PadFreeCount..VertCount-1] are the pinned region.
                if (state.SkinnedPositions != null && state.PadFreeCount > 0)
                {
                    int padFree = state.PadFreeCount;
                    int total   = state.VertCount;
                    for (int pi = padFree; pi < total; pi++)
                    {
                        state.Position[pi]     = state.SkinnedPositions[pi];
                        state.PredPosition[pi] = state.SkinnedPositions[pi];
                        state.Velocity[pi]     = Vector3.zero;
                    }
                }

                if (state.Params.ClothToCloth)
                {
                    // Managed cloth-to-cloth collision removed. 
                    // Native/GPU paths should handle this if implemented there.
                }

                WriteMesh(state);

                // Count down warmup
                if (state.WarmupFramesRemaining > 0)
                    state.WarmupFramesRemaining--;
            }
        }




        // ================================================================== //
        // xPBD substep  (Macklin et al. 2016 — Extended Position-Based Dynamics)
        //
        // Key difference from VBD: constraints work on a separate PredPosition
        // buffer, not on Position directly.  Velocity is derived from the total
        // position change (pos_after - pos_before), so gravity accumulates
        // correctly across frames and cloth actually drapes.
        //
        // Compliance α = 1/k.  Tilded compliance α̃ = α/dt² makes constraints
        // timestep-invariant — no parameter re-tuning needed when Substeps change.
        // ================================================================== //

        private const float MaxCompressionRatio = 0.35f;


        // ================================================================== //
        // GPU xPBD — all substeps on GPU, single upload/readback
        // ================================================================== //
        private void SimulateAllSubstepsGPU(ClothMeshState s, float dt, int substeps)
        {
            int n = s.VertCount;

            Array.Copy(s.Position, s.PrevPosition, n);

            float g = s.Params.Gravity * s.Params.Weight;
            float subDt = dt / Mathf.Max(1, substeps);
            float tildedCompliance = 1f / Mathf.Max(1e-6f, s.Params.StretchStiffness * subDt * subDt);
            float bendCompliance   = 1f / Mathf.Max(1e-6f, s.Params.BendStiffness * subDt * subDt);
            float thick = s.Params.Thickness;
            float pressScale = 1f - s.Params.Compression * MaxCompressionRatio;

            // Damping (warmup-aware)
            float dampBase = s.Params.Damping;
            if (s.WarmupFramesRemaining > 0)
                dampBase = Mathf.Lerp(dampBase, 30f,
                    s.WarmupFramesRemaining / (float)ClothMeshState.WarmupDuration);
            float damp     = Mathf.Clamp01(1f - dampBase * subDt);
            float maxSpeed = s.WarmupFramesRemaining > 0 ? 2f : 15f;
            float frictionZone = thick * 2.5f;

            int iterations = Mathf.Max(1, Mathf.RoundToInt(s.Params.Iterations));

            // Build colliders for GPU
            BuildNativeColliders();

            // SDF state
            bool useSDF = UseSDFColliders && _sdfProxy != null && _sdfProxy.IsReady;
            ComputeBuffer sdfBuf = useSDF ? _sdfProxy.GPUSDFBuffer : null;

            _gpuClothSolver.Simulate(
                s.Position, s.Velocity, s.InvMass, s.IsPinned,
                n, s.PadFreeCount,
                _nColliders, _mirrorColCount,
                g, dt, substeps, iterations,
                tildedCompliance, bendCompliance,
                pressScale, thick,
                damp, maxSpeed, frictionZone,
                useSDF && sdfBuf != null, sdfBuf,
                useSDF ? _sdfProxy.SDFOriginX : 0f,
                useSDF ? _sdfProxy.SDFOriginY : 0f,
                useSDF ? _sdfProxy.SDFOriginZ : 0f,
                useSDF ? _sdfProxy.SDFResX : 0,
                useSDF ? _sdfProxy.SDFResY : 0,
                useSDF ? _sdfProxy.SDFResZ : 0,
                useSDF ? _sdfProxy.SDFCellSize : 0f,
                useSDF ? _sdfProxy.SDFMaxDist : 0f);

            // HARD LOCK: GPU GetData may have overwritten pinned positions with
            // whatever the compute shader stored. Force them back to exact LBS.
            if (s.SkinnedPositions != null)
            {
                int padFree = s.PadFreeCount;
                int total   = s.VertCount;
                for (int i = padFree; i < total; i++)
                {
                    s.Position[i]     = s.SkinnedPositions[i];
                    s.PredPosition[i] = s.SkinnedPositions[i];
                    s.Velocity[i]     = Vector3.zero;
                }
            }
        }

        // ================================================================== //
        // Native xPBD substep — delegates hot loops to StudioModsNative.dll
        // ================================================================== //
        private void SimulateStepNative(ClothMeshState s, float dt)
        {
            int totalN = s.VertCount;
            int freeN = s.PadFreeCount > 0 ? s.PadFreeCount : totalN;
            EnsureNativeBuffers(s);
            SyncToNative(s);

            // 1. Predict
            float g = s.Params.Gravity * s.Params.Weight;
            NativeBridge.Cloth_Predict(_nPos, _nVel, _nPred, _nInvMass, freeN, g, dt);

            // 2. Constraints + Collision dentro del loop
            float tildedCompliance = 1f / Mathf.Max(1e-6f, s.Params.StretchStiffness * dt * dt);
            float bendCompliance   = 1f / Mathf.Max(1e-6f, s.Params.BendStiffness * dt * dt);
            float thick = s.Params.Thickness;
            bool useSDF = UseSDFColliders && _sdfProxy != null && _sdfProxy.IsReady;

            int iterations = Mathf.Max(1, Mathf.RoundToInt(s.Params.Iterations));

            // Pre-construir colliders nativos UNA sola vez antes del loop
            BuildNativeColliders();

            for (int iter = 0; iter < iterations; iter++)
            {
                // Edges
                if (s.EdgeColorGroups != null)
                {
                    float pressScale = 1f - s.Params.Compression * MaxCompressionRatio;
                    foreach (int[] group in s.EdgeColorGroups)
                    {
                        int[]   groupEdges   = new int[group.Length * 2];
                        float[] groupRestLen = new float[group.Length];
                        for (int gi = 0; gi < group.Length; gi++)
                        {
                            int eIdx = group[gi];
                            groupEdges[gi * 2]     = s.Edges[eIdx * 2];
                            groupEdges[gi * 2 + 1] = s.Edges[eIdx * 2 + 1];
                            groupRestLen[gi]        = s.RestEdgeLen[eIdx];
                        }
                        NativeBridge.Cloth_SolveEdges(_nPred, _nInvMass, _nIsPinned,
                            groupEdges, groupRestLen, group.Length, tildedCompliance, pressScale);
                    }
                }
                else
                {
                    float pressScale = 1f - s.Params.Compression * MaxCompressionRatio;
                    NativeBridge.Cloth_SolveEdges(_nPred, _nInvMass, _nIsPinned,
                        s.Edges, s.RestEdgeLen, s.Edges.Length / 2, tildedCompliance, pressScale);
                }

                // Bends
                if (s.BendPairs != null && s.BendPairs.Length > 0)
                {
                    if (s.BendColorGroups != null)
                    {
                        foreach (int[] group in s.BendColorGroups)
                        {
                            int[]   groupBends      = new int[group.Length * 2];
                            float[] groupRestBendLen = new float[group.Length];
                            for (int gi = 0; gi < group.Length; gi++)
                            {
                                int bIdx = group[gi];
                                groupBends[gi * 2]     = s.BendPairs[bIdx * 2];
                                groupBends[gi * 2 + 1] = s.BendPairs[bIdx * 2 + 1];
                                groupRestBendLen[gi]    = s.RestBendLen[bIdx];
                            }
                            NativeBridge.Cloth_SolveBends(_nPred, _nInvMass, _nIsPinned,
                                groupBends, groupRestBendLen, group.Length, bendCompliance);
                        }
                    }
                    else
                    {
                        NativeBridge.Cloth_SolveBends(_nPred, _nInvMass, _nIsPinned,
                            s.BendPairs, s.RestBendLen, s.BendPairs.Length / 2, bendCompliance);
                    }
                }

                // Collision cápsulas (native)
                if (_mirrorColCount > 0)
                    NativeBridge.Cloth_SolveCollision(
                        _nPred, _nInvMass, freeN, _nColliders, _mirrorColCount, thick);

                // Collision SDF (native)
                if (useSDF)
                {
                    NativeBridge.SDF_CollideVertices(
                        _nPred, _nInvMass, freeN, 
                        _sdfProxy.GetSDFDataInternal(),
                        _sdfProxy.SDFResX, _sdfProxy.SDFResY, _sdfProxy.SDFResZ,
                        _sdfProxy.SDFOriginX, _sdfProxy.SDFOriginY, _sdfProxy.SDFOriginZ,
                        1f / _sdfProxy.SDFCellSize, _sdfProxy.SDFMaxDist,
                        thick, 1.0f);
                }
            }

            // 3. Commit
            float inv_dt   = 1f / dt;
            float dampBase = s.Params.Damping;
            if (s.WarmupFramesRemaining > 0)
                dampBase = Mathf.Lerp(dampBase, 30f,
                    s.WarmupFramesRemaining / (float)ClothMeshState.WarmupDuration);
            float damp     = Mathf.Clamp01(1f - dampBase * dt);
            float maxSpeed = s.WarmupFramesRemaining > 0 ? 2f : 15f;

            NativeBridge.Cloth_Commit(_nPos, _nVel, _nPred, _nInvMass, freeN, inv_dt, damp, maxSpeed);
            SyncFromNative(s);

        }
        private void WriteMesh(ClothMeshState s)
        {
            if (s.WorkMesh == null || s.Filter == null || s.LocalVerts == null || s.VisualToSimMap == null) return;

            Transform tf  = s.Renderer != null ? s.Renderer.transform : transform;
            Matrix4x4 w2l = tf.worldToLocalMatrix;
            int       origVc  = s.OriginalVertCount;
            Vector3[] pos  = s.Position;
            Vector3[] lv   = s.LocalVerts;

            if (origVc > 512)
            {
                Parallel.For(0, origVc, i => { 
                    int simIdx = s.VisualToSimMap[i];
                    lv[i] = w2l.MultiplyPoint3x4(pos[simIdx]); 
                });
            }
            else
            {
                for (int i = 0; i < origVc; i++)
                {
                    int simIdx = s.VisualToSimMap[i];
                    lv[i] = w2l.MultiplyPoint3x4(pos[simIdx]);
                }
            }

            s.WorkMesh.vertices = lv;
            s.WorkMesh.RecalculateBounds();   // CRITICAL: without this Unity frustum-culls the mesh
            s.WorkMesh.RecalculateNormals();

            // Tangents are expensive and rarely needed for cloth materials — skip by default
            if (s.Params.NeedsTangents)
                s.WorkMesh.RecalculateTangents();

            // MeshCollider cook is very expensive — throttle to every N frames.
            // The collider is only used for mouse picking in the editor, not for physics.
            _meshColliderFrame++;
            if (s.ClothCollider != null && (_meshColliderFrame % MeshColliderUpdateInterval == 0))
            {
                s.ClothCollider.sharedMesh = null;
                s.ClothCollider.sharedMesh = s.WorkMesh;
            }
        }

        /// <summary>
        /// Per-frame LBS skinning for ALL vertices + per-vertex bone-follow.
        /// Layout contract (post-SetupMeshState):
        ///   [0 .. FreeVertCount-1]   = free simulated particles
        ///   [FreeVertCount .. PadFreeCount-1] = dummy padding (InvMass=0, skip)
        ///   [PadFreeCount .. VertCount-1]     = PINNED particles (never simulated)
        /// This function:
        ///   1. Computes LBS for every particle into SkinnedPositions.
        ///   2. IMMEDIATELY hard-locks pinned positions to LBS (before any solver runs).
        ///   3. Applies bone-follow delta only to free particles.
        /// </summary>
        private void SkinAndFollowBones(ClothMeshState s)
        {
            if (s.SrcBindVerts == null || s.BindPoses == null ||
                s.SkinBones == null || s.VertexBoneWeights == null) return;

            int n         = s.VertCount;
            int freeN     = s.FreeVertCount > 0 ? s.FreeVertCount : n;
            int padFreeN  = s.PadFreeCount  > 0 ? s.PadFreeCount  : n;
            int boneCount = s.SkinBones.Length;

            // Ensure per-mesh LBS caches are allocated
            if (s.SkinnedPositions == null || s.SkinnedPositions.Length < n)
                s.SkinnedPositions = new Vector3[n];
            if (s.PrevSkinnedPositions == null || s.PrevSkinnedPositions.Length < n)
                s.PrevSkinnedPositions = new Vector3[n];

            bool hasPrev = true; // buffers are always primed by WarmUpClothState

            // Build bone matrices once per frame
            if (_skinMats == null || _skinMats.Length < boneCount)
                _skinMats = new Matrix4x4[boneCount];
            for (int b = 0; b < boneCount; b++)
            {
                _skinMats[b] = (s.SkinBones[b] != null && b < s.BindPoses.Length)
                    ? s.SkinBones[b].localToWorldMatrix * s.BindPoses[b]
                    : Matrix4x4.identity;
            }

            Matrix4x4[] mats = _skinMats;
            Vector3[]   prev = s.PrevSkinnedPositions;
            Vector3[]   curr = s.SkinnedPositions;

            for (int i = 0; i < n; i++)
            {
                Vector3 v = s.SrcBindVerts[i];
                BoneWeight bw = s.VertexBoneWeights[i];

                Vector3 skinned =
                    (Vector3)mats[bw.boneIndex0].MultiplyPoint3x4(v) * bw.weight0 +
                    (Vector3)mats[bw.boneIndex1].MultiplyPoint3x4(v) * bw.weight1 +
                    (Vector3)mats[bw.boneIndex2].MultiplyPoint3x4(v) * bw.weight2 +
                    (Vector3)mats[bw.boneIndex3].MultiplyPoint3x4(v) * bw.weight3;

                curr[i] = skinned;

                bool isDummy  = (i >= freeN  && i < padFreeN);
                bool isPinned = (i >= padFreeN);  // pinned region starts at PadFreeCount

                if (isPinned)
                {
                    // HARD LOCK: set directly to LBS — solver never touches these indices,
                    // but we set all three buffers so WriteMesh always gets the right value.
                    s.Position[i]     = skinned;
                    s.PredPosition[i] = skinned;
                    s.Velocity[i]     = Vector3.zero;
                }
                else if (!isDummy && hasPrev && s.InvMass[i] > 0f)
                {
                    // Free particle bone-follow: advect by LBS delta so the cloth
                    // moves with the character skeleton.
                    Vector3 delta = skinned - prev[i];
                    if (delta.sqrMagnitude > 1e-12f)
                    {
                        s.Position[i]     += delta;
                        s.PrevPosition[i] += delta;
                    }
                }
                // dummy-padding and InvMass=0-frozen: intentionally unchanged.
            }

            // Rotate buffers: current frame becomes previous for next frame
            Array.Copy(curr, prev, n);
        }

        /// <summary>
        /// After simulation: blend partially-pinned vertices toward their LBS position.
        /// Only operates on FREE particles (indices 0..FreeVertCount-1).
        /// </summary>
        private void ApplyPinBlend(ClothMeshState s)
        {
            if (s.SkinnedPositions == null || s.PinBlend == null) return;
            int freeN = s.FreeVertCount > 0 ? s.FreeVertCount : s.VertCount;
            for (int i = 0; i < freeN; i++)
            {
                if (s.IsPinned[i]) continue;

                float blend = s.PinBlend[i];
                if (blend > 0f && blend < 1f)
                {
                    s.Position[i] = Vector3.Lerp(s.Position[i], s.SkinnedPositions[i], blend);
                    s.Velocity[i] *= (1f - blend);
                }
            }
        }

        /// <summary>
        /// Recomputes pin state from selected bone names.
        /// PinBlend is computed from the sum of bone weights to selected bones:
        ///   blend >= 1 → fully pinned (IsPinned=true, InvMass=0)
        ///   0 < blend < 1 → blend zone (simulated but result lerped toward LBS)
        ///   blend == 0 → fully simulated
        /// </summary>
        public void RecomputePinsFromSelectedBone(ClothMeshState state)
        {
            if (state == null || state.IsPinned == null || chaCtrl == null) return;
            int n = state.VertCount;

            // Allocate PinBlend if needed
            if (state.PinBlend == null || state.PinBlend.Length != n)
                state.PinBlend = new float[n];

            for (int i = 0; i < n; i++)
            {
                state.IsPinned[i] = false;
                state.PinBlend[i] = 0f;
            }

            if (state.PinSourceBoneNames != null && state.PinSourceBoneNames.Count > 0 &&
                state.SkinBones != null && state.VertexBoneWeights != null)
            {
                var selectedBoneIndices = new HashSet<int>();
                for (int b = 0; b < state.SkinBones.Length; b++)
                {
                    Transform bone = state.SkinBones[b];
                    if (bone == null) continue;
                    for (int sbi = 0; sbi < state.PinSourceBoneNames.Count; sbi++)
                    {
                        string selectedName = state.PinSourceBoneNames[sbi];
                        if (string.IsNullOrEmpty(selectedName)) continue;
                        if (string.Equals(bone.name, selectedName, StringComparison.OrdinalIgnoreCase))
                        {
                            selectedBoneIndices.Add(b);
                            break;
                        }
                    }
                }

                if (selectedBoneIndices.Count > 0)
                {
                    for (int i = 0; i < n; i++)
                    {
                        BoneWeight bw = state.VertexBoneWeights[i];
                        // Sum weights from all selected bones → natural smooth blend
                        float blend = 0f;
                        if (selectedBoneIndices.Contains(bw.boneIndex0)) blend += bw.weight0;
                        if (selectedBoneIndices.Contains(bw.boneIndex1)) blend += bw.weight1;
                        if (selectedBoneIndices.Contains(bw.boneIndex2)) blend += bw.weight2;
                        if (selectedBoneIndices.Contains(bw.boneIndex3)) blend += bw.weight3;

                        state.PinBlend[i] = blend;
                        state.IsPinned[i] = blend >= 0.999f;
                    }
                }
            }

            float mass = 1f / Mathf.Max(1, n);
            for (int i = 0; i < n; i++)
            {
                state.Mass[i]    = mass;
                state.InvMass[i] = state.IsPinned[i] ? 0f : 1f / mass;
                if (state.IsPinned[i] && state.Velocity != null)
                    state.Velocity[i] = Vector3.zero;
            }

            // Correct SrcBindVerts so that LBS(correctedVert) == current Position.
            // Run once at pin time — SkinPinnedVertices then uses the corrected verts each frame.
            if (state.SrcBindVerts != null && state.BindPoses != null && state.SkinBones != null)
            {
                int boneCount = state.SkinBones.Length;
                Matrix4x4[] pinSkinMats = new Matrix4x4[boneCount];
                for (int b = 0; b < boneCount; b++)
                {
                    pinSkinMats[b] = (state.SkinBones[b] != null && b < state.BindPoses.Length)
                        ? state.SkinBones[b].localToWorldMatrix * state.BindPoses[b]
                        : Matrix4x4.identity;
                }

                for (int i = 0; i < n; i++)
                {
                    if (state.PinBlend[i] <= 0f) continue;

                    BoneWeight bw = state.VertexBoneWeights[i];
                    Matrix4x4 m0 = pinSkinMats[bw.boneIndex0];
                    Matrix4x4 m1 = pinSkinMats[bw.boneIndex1];
                    Matrix4x4 m2 = pinSkinMats[bw.boneIndex2];
                    Matrix4x4 m3 = pinSkinMats[bw.boneIndex3];

                    // Build weighted blended skinning matrix
                    Matrix4x4 blended = new Matrix4x4();
                    for (int e = 0; e < 16; e++)
                        blended[e] = m0[e] * bw.weight0 + m1[e] * bw.weight1 +
                                     m2[e] * bw.weight2 + m3[e] * bw.weight3;

                    // Invert: find the bind-space vertex that produces current world position
                    Matrix4x4 inv = blended.inverse;
                    state.SrcBindVerts[i] = inv.MultiplyPoint3x4(state.Position[i]);
                }
            }
        }

        private void OnDrawGizmos()
        {
            if (entries == null || entries.Count == 0) return;

            foreach (ClothPhysicsEntry entry in entries)
            {
                if (entry == null || entry.Meshes == null) continue;
                foreach (ClothMeshState mesh in entry.Meshes)
                {
                    if (mesh == null || !mesh.ShowPinBoneGizmos || mesh.SkinBones == null) continue;
                    if (mesh.PinSourceBoneNames == null || mesh.PinSourceBoneNames.Count == 0) continue;

                    for (int i = 0; i < mesh.SkinBones.Length; i++)
                    {
                        Transform bone = mesh.SkinBones[i];
                        if (bone == null || !ContainsBoneName(mesh.PinSourceBoneNames, bone.name)) continue;

                        // Determine pin strength for this bone (blend of all influenced vertices)
                        Gizmos.color = new Color(0.1f, 1f, 0.4f, 0.95f); // green = pinned
                        Gizmos.DrawSphere(bone.position, 0.018f);

                        // Draw bone segment to parent
                        Transform parent = bone.parent;
                        if (parent != null)
                        {
                            Gizmos.color = new Color(0.1f, 0.8f, 1f, 0.7f);
                            Gizmos.DrawLine(parent.position, bone.position);
                        }

                        // Draw child stubs for clarity
                        for (int c = 0; c < bone.childCount; c++)
                        {
                            Transform child = bone.GetChild(c);
                            if (child == null) continue;
                            Gizmos.color = new Color(0.1f, 1f, 0.4f, 0.35f);
                            Gizmos.DrawLine(bone.position, child.position);
                        }
                    }
                }
            }
        }

        private static bool ContainsBoneName(List<string> selectedNames, string boneName)
        {
            if (selectedNames == null || string.IsNullOrEmpty(boneName)) return false;
            for (int i = 0; i < selectedNames.Count; i++)
            {
                string v = selectedNames[i];
                if (!string.IsNullOrEmpty(v) && string.Equals(v, boneName, StringComparison.OrdinalIgnoreCase))
                    return true;
            }
            return false;
        }
        private static void UpdateWorldBounds(ClothMeshState s)
        {
            if (s.VertCount == 0) return;
            Bounds b = new Bounds(s.Position[0], Vector3.zero);
            for (int i = 1; i < s.VertCount; i++) b.Encapsulate(s.Position[i]);
            s.WorldBounds = b;
        }
        private bool SetupMeshState(ClothMeshState state)
        {
            SkinnedMeshRenderer smr = state.Renderer;
            if (smr == null || smr.sharedMesh == null) return false;

            Mesh src = smr.sharedMesh;

            Mesh baked = new Mesh();
            smr.BakeMesh(baked);
            baked.name = src.name + "_ClothPhysics";

            state.WorkMesh = baked;
            state.WorkMesh.MarkDynamic(); 
            int originalVertCount = baked.vertices.Length;
            state.OriginalVertCount = originalVertCount;

            Transform tf       = smr.transform;
            Vector3[] bakedV   = baked.vertices;
            int[]     srcTris  = baked.triangles;
            state.SkinBones    = smr.bones;
            Matrix4x4[] bindPoses = src.bindposes;
            Vector3[]   srcVerts  = src.vertices;
            BoneWeight[] srcBoneWeights = src.boneWeights;

            // 1. Initial Welding / Decimation
            Vector3[] worldBaked = new Vector3[originalVertCount];
            for (int i = 0; i < originalVertCount; i++) worldBaked[i] = tf.TransformPoint(bakedV[i]);
            
            int[] vertexMapping = WeldVertices(worldBaked, srcTris, state.Params.DecimationThreshold);
            
            List<int> uniqueOrigIndices = new List<int>();
            int[] origToUnique = new int[originalVertCount];
            for (int i = 0; i < originalVertCount; i++)
            {
                if (vertexMapping[i] == i)
                {
                    origToUnique[i] = uniqueOrigIndices.Count;
                    uniqueOrigIndices.Add(i);
                }
                else origToUnique[i] = -1;
            }
            
            for (int i = 0; i < originalVertCount; i++)
            {
                int rep = vertexMapping[i];
                while (vertexMapping[rep] != rep) rep = vertexMapping[rep];
                origToUnique[i] = origToUnique[rep];
            }
            
            int uniqueCount = uniqueOrigIndices.Count;
            
            // 2. Identify Pinned Particles
            bool[] uniqueIsPinned = new bool[uniqueCount];
            if (state.PinSourceBoneNames != null && state.PinSourceBoneNames.Count > 0 && state.SkinBones != null && srcBoneWeights != null)
            {
                var selectedBoneIndices = new System.Collections.Generic.HashSet<int>();
                for (int b = 0; b < state.SkinBones.Length; b++)
                {
                    Transform bone = state.SkinBones[b];
                    if (bone == null) continue;
                    for (int sbi = 0; sbi < state.PinSourceBoneNames.Count; sbi++)
                    {
                        if (string.Equals(bone.name, state.PinSourceBoneNames[sbi], StringComparison.OrdinalIgnoreCase))
                        {
                            selectedBoneIndices.Add(b); break;
                        }
                    }
                }

                if (selectedBoneIndices.Count > 0)
                {
                    for (int i = 0; i < uniqueCount; i++)
                    {
                        int origIdx = uniqueOrigIndices[i];
                        if (origIdx >= srcBoneWeights.Length) continue;
                        BoneWeight bw = srcBoneWeights[origIdx];
                        float blend = 0f;
                        if (selectedBoneIndices.Contains(bw.boneIndex0)) blend += bw.weight0;
                        if (selectedBoneIndices.Contains(bw.boneIndex1)) blend += bw.weight1;
                        if (selectedBoneIndices.Contains(bw.boneIndex2)) blend += bw.weight2;
                        if (selectedBoneIndices.Contains(bw.boneIndex3)) blend += bw.weight3;
                        if (blend >= 0.999f) uniqueIsPinned[i] = true;
                    }
                }
            }
            
            // 3. Separate Free and Pinned
            List<int> freeIndices = new List<int>();
            List<int> pinnedIndices = new List<int>();
            for (int i = 0; i < uniqueCount; i++)
            {
                if (uniqueIsPinned[i]) pinnedIndices.Add(uniqueOrigIndices[i]);
                else freeIndices.Add(uniqueOrigIndices[i]);
            }
            
            int numFree = freeIndices.Count;
            int numPinned = pinnedIndices.Count;
            int padFree = Mathf.CeilToInt(numFree / 256f) * 256;
            if (padFree == 0 && numPinned > 0) padFree = 256; 
            
            state.FreeVertCount = numFree;
            state.PadFreeCount = padFree;
            int totalSim = padFree + numPinned;
            state.VertCount = totalSim;
            
            state.VisualToSimMap = new int[originalVertCount];
            int[] simToOrigMap = new int[totalSim];
            int[] origToSimMap = new int[originalVertCount];
            int currentSimIdx = 0;
            
            for(int i=0; i<numFree; i++) {
                int orig = freeIndices[i];
                origToSimMap[orig] = currentSimIdx;
                simToOrigMap[currentSimIdx++] = orig;
            }
            
            int dummyOrig = numFree > 0 ? freeIndices[0] : (numPinned > 0 ? pinnedIndices[0] : 0);
            for(int i=numFree; i<padFree; i++) {
                simToOrigMap[currentSimIdx++] = dummyOrig;
            }
            
            for(int i=0; i<numPinned; i++) {
                int orig = pinnedIndices[i];
                origToSimMap[orig] = currentSimIdx;
                simToOrigMap[currentSimIdx++] = orig;
            }
            
            for (int i = 0; i < originalVertCount; i++) {
                int rep = vertexMapping[i];
                while (vertexMapping[rep] != rep) rep = vertexMapping[rep];
                state.VisualToSimMap[i] = origToSimMap[rep];
            }
            
            state.Position     = new Vector3[totalSim];
            state.Velocity     = new Vector3[totalSim];
            state.PredPosition = new Vector3[totalSim];
            state.PrevPosition = new Vector3[totalSim];
            state.IsPinned     = new bool[totalSim];
            state.Mass         = new float[totalSim];
            state.InvMass      = new float[totalSim];
            state.VertexBoneWeights = new BoneWeight[totalSim];
            state.SrcBindVerts = new Vector3[totalSim];
            state.BindPoses    = bindPoses;
            
            float massPerFree = 1f / Mathf.Max(1, numFree);
            for (int i = 0; i < totalSim; i++) {
                int orig = simToOrigMap[i];
                state.Position[i] = worldBaked[orig];
                
                bool isDummy = (i >= numFree && i < padFree);
                bool isPinnedPart = (i >= padFree);
                
                state.IsPinned[i] = isPinnedPart;
                if (isPinnedPart || isDummy) {
                    state.Mass[i] = massPerFree;
                    state.InvMass[i] = 0f;
                } else {
                    state.Mass[i] = massPerFree;
                    state.InvMass[i] = 1f / massPerFree;
                }
                
                if (srcBoneWeights != null && orig < srcBoneWeights.Length)
                    state.VertexBoneWeights[i] = srcBoneWeights[orig];
                if (srcVerts != null && orig < srcVerts.Length)
                    state.SrcBindVerts[i] = srcVerts[orig];
            }
            
            int[] newTris = new int[srcTris.Length];
            for (int i = 0; i < srcTris.Length; i++) {
                newTris[i] = state.VisualToSimMap[srcTris[i]];
            }
            
            state.Tris = FilterValidTriangles(newTris, totalSim);
            
            if (bindPoses != null && srcVerts != null && state.SkinBones != null && state.SkinBones.Length > 0)
            {
                Matrix4x4[] skinMats = new Matrix4x4[state.SkinBones.Length];
                for (int b = 0; b < state.SkinBones.Length; b++)
                {
                    skinMats[b] = (state.SkinBones[b] != null && b < bindPoses.Length)
                        ? state.SkinBones[b].localToWorldMatrix * bindPoses[b]
                        : Matrix4x4.identity;
                }

                for (int i = 0; i < totalSim; i++)
                {
                    Vector3 v = state.SrcBindVerts[i];
                    BoneWeight bw = state.VertexBoneWeights[i];
                    state.Position[i] =
                        (Vector3)skinMats[bw.boneIndex0].MultiplyPoint3x4(v) * bw.weight0 +
                        (Vector3)skinMats[bw.boneIndex1].MultiplyPoint3x4(v) * bw.weight1 +
                        (Vector3)skinMats[bw.boneIndex2].MultiplyPoint3x4(v) * bw.weight2 +
                        (Vector3)skinMats[bw.boneIndex3].MultiplyPoint3x4(v) * bw.weight3;
                }
            }

            Array.Copy(state.Position, state.PrevPosition, totalSim);
            Array.Copy(state.Position, state.PredPosition, totalSim);

            if (chaCtrl != null)
            {
                state.RestBodyLocalPos = new Vector3[totalSim];
                for (int i = 0; i < totalSim; i++)
                    state.RestBodyLocalPos[i] = chaCtrl.transform.InverseTransformPoint(state.Position[i]);
            }

            BuildEdges(state, state.Tris, state.Position);
            BuildBends(state, state.Tris, state.Position);

            GameObject go = smr.gameObject;
            state.OriginalMaterials = smr.sharedMaterials;
            smr.enabled = false;

            if (state.Filter == null)  state.Filter  = go.AddComponent<MeshFilter>();
            if (state.MeshRend == null) state.MeshRend = go.AddComponent<MeshRenderer>();
            if (state.ClothCollider == null)
                state.ClothCollider = go.GetComponent<MeshCollider>() ?? go.AddComponent<MeshCollider>();

            state.Filter.sharedMesh        = state.WorkMesh;
            state.MeshRend.sharedMaterials = state.OriginalMaterials;
            state.MeshRend.shadowCastingMode       = state.Renderer.shadowCastingMode;
            state.MeshRend.receiveShadows          = state.Renderer.receiveShadows;

            state.ClothCollider.convex = false;
            state.ClothCollider.sharedMesh = null;
            state.ClothCollider.sharedMesh = state.WorkMesh;
            state.ClothCollider.enabled = true;

            state.LocalVerts = new Vector3[originalVertCount];
            UpdateWorldBounds(state);
            return true;
        }

        private void TeardownMeshState(ClothMeshState state)
        {
            if (state.Renderer != null)
            {
                state.Renderer.enabled = true;
                if (state.OriginalMaterials != null)
                    state.Renderer.sharedMaterials = state.OriginalMaterials;
            }

            if (state.MeshRend != null) Destroy(state.MeshRend);
            if (state.ClothCollider != null) Destroy(state.ClothCollider);
            if (state.Filter   != null) Destroy(state.Filter);
            if (state.WorkMesh != null) Destroy(state.WorkMesh);

            state.MeshRend = null;
            state.ClothCollider = null;
            state.Filter   = null;
            state.WorkMesh = null;
        }
        private static int[] WeldVertices(Vector3[] positions, int[] triangles, float threshold)
        {
            int n = positions.Length;
            int[] mapping = new int[n];
            for (int i = 0; i < n; i++) mapping[i] = i;

            float threshSq = threshold * threshold;

            for (int i = 0; i < n; i++)
            {
                if (mapping[i] != i) continue;  // already merged
                
                Vector3 pi = positions[i];
                for (int j = i + 1; j < n; j++)
                {
                    if (mapping[j] != j) continue;  // j already merged
                    if ((positions[j] - pi).sqrMagnitude <= threshSq)
                    {
                        mapping[j] = i;
                    }
                }
            }

            return mapping;
        }

        private static int[] FilterValidTriangles(int[] tris, int vertexCount)
        {
            var filtered = new List<int>(tris.Length);
            for (int i = 0; i + 2 < tris.Length; i += 3)
            {
                int a = tris[i];
                int b = tris[i + 1];
                int c = tris[i + 2];
                if (a < 0 || b < 0 || c < 0) continue;
                if (a >= vertexCount || b >= vertexCount || c >= vertexCount) continue;
                if (a == b || b == c || c == a) continue;
                filtered.Add(a);
                filtered.Add(b);
                filtered.Add(c);
            }
            return filtered.ToArray();
        }
        private static void BuildEdges(ClothMeshState s, int[] tris, Vector3[] worldPosRest)
        {
            var edgeSet = new HashSet<long>();
            var edgeList = new List<int>();
            var restLens  = new List<float>();

            for (int t = 0; t < tris.Length; t += 3)
            {
                AddEdge(tris[t], tris[t + 1], edgeSet, edgeList, restLens, worldPosRest);
                AddEdge(tris[t + 1], tris[t + 2], edgeSet, edgeList, restLens, worldPosRest);
                AddEdge(tris[t + 2], tris[t], edgeSet, edgeList, restLens, worldPosRest);
            }

            s.Edges       = edgeList.ToArray();
            s.RestEdgeLen = restLens.ToArray();
            BuildEdgeColorGroups(s);
        }

        private static void BuildBends(ClothMeshState s, int[] tris, Vector3[] worldPosRest)
        {
            var edgeToOpposites = new Dictionary<long, List<int>>();

            for (int t = 0; t < tris.Length; t += 3)
            {
                int a = tris[t];
                int b = tris[t + 1];
                int c = tris[t + 2];

                AddBendEdge(edgeToOpposites, a, b, c);
                AddBendEdge(edgeToOpposites, b, c, a);
                AddBendEdge(edgeToOpposites, c, a, b);
            }

            var bendList = new List<int>();
            var restLens = new List<float>();

            foreach (var kv in edgeToOpposites)
            {
                if (kv.Value.Count != 2) continue;

                int oppA = kv.Value[0];
                int oppB = kv.Value[1];
                if (oppA == oppB) continue;

                bendList.Add(oppA);
                bendList.Add(oppB);
                restLens.Add((worldPosRest[oppA] - worldPosRest[oppB]).magnitude);
            }

            s.BendPairs   = bendList.ToArray();
            s.RestBendLen = restLens.ToArray();
            BuildBendColorGroups(s);
        }

        // ── Graph coloring: partitions edge/bend pairs into independent groups ──
        // Two pairs are "adjacent" if they share a vertex. Greedy coloring ensures
        // that within a color group no two pairs touch the same vertex — making
        // Parallel.For safe without locks or atomic operations.
        private static void BuildEdgeColorGroups(ClothMeshState s)
        {
            int pairCount = s.Edges.Length / 2;
            if (pairCount == 0) { s.EdgeColorGroups = Array.Empty<int[]>(); return; }
            int vertCount = s.VertCount;
            int[] pairColor = new int[pairCount];
            ulong[] vertColorMask = new ulong[vertCount];
            List<int>[] overflowColors = new List<int>[vertCount];

            int maxColor = 0;
            for (int p = 0; p < pairCount; p++)
            {
                int vi = s.Edges[p * 2];
                int vj = s.Edges[p * 2 + 1];
                int c  = 0;
                while (VertexHasColor(vertColorMask, overflowColors, vi, c) ||
                       VertexHasColor(vertColorMask, overflowColors, vj, c)) c++;
                pairColor[p]  = c;
                VertexAddColor(vertColorMask, overflowColors, vi, c);
                VertexAddColor(vertColorMask, overflowColors, vj, c);
                if (c > maxColor) maxColor = c;
            }

            int colorCount = maxColor + 1;
            int[] groupSizes = new int[colorCount];
            for (int p = 0; p < pairCount; p++) groupSizes[pairColor[p]]++;

            s.EdgeColorGroups = new int[colorCount][];
            for (int c = 0; c < colorCount; c++)
                s.EdgeColorGroups[c] = new int[groupSizes[c]];

            Array.Clear(groupSizes, 0, colorCount); // reuse as write cursors
            for (int p = 0; p < pairCount; p++)
            {
                int c = pairColor[p];
                s.EdgeColorGroups[c][groupSizes[c]++] = p;
            }
        }

        private static void BuildBendColorGroups(ClothMeshState s)
        {
            if (s.BendPairs == null || s.BendPairs.Length == 0) { s.BendColorGroups = null; return; }
            int pairCount = s.BendPairs.Length / 2;
            int vertCount = s.VertCount;
            int[] pairColor = new int[pairCount];
            ulong[] vertColorMask = new ulong[vertCount];
            List<int>[] overflowColors = new List<int>[vertCount];

            int maxColor = 0;
            for (int p = 0; p < pairCount; p++)
            {
                int vi = s.BendPairs[p * 2];
                int vj = s.BendPairs[p * 2 + 1];
                int c  = 0;
                while (VertexHasColor(vertColorMask, overflowColors, vi, c) ||
                       VertexHasColor(vertColorMask, overflowColors, vj, c)) c++;
                pairColor[p]  = c;
                VertexAddColor(vertColorMask, overflowColors, vi, c);
                VertexAddColor(vertColorMask, overflowColors, vj, c);
                if (c > maxColor) maxColor = c;
            }

            int colorCount = maxColor + 1;
            int[] groupSizes = new int[colorCount];
            for (int p = 0; p < pairCount; p++) groupSizes[pairColor[p]]++;

            s.BendColorGroups = new int[colorCount][];
            for (int c = 0; c < colorCount; c++)
                s.BendColorGroups[c] = new int[groupSizes[c]];

            Array.Clear(groupSizes, 0, colorCount);
            for (int p = 0; p < pairCount; p++)
            {
                int c = pairColor[p];
                s.BendColorGroups[c][groupSizes[c]++] = p;
            }
        }

        private static bool VertexHasColor(ulong[] colorMask, List<int>[] overflowColors, int vertexIndex, int color)
        {
            if (color < 64) return (colorMask[vertexIndex] & (1UL << color)) != 0;
            List<int> overflow = overflowColors[vertexIndex];
            return overflow != null && overflow.Contains(color);
        }

        private static void VertexAddColor(ulong[] colorMask, List<int>[] overflowColors, int vertexIndex, int color)
        {
            if (color < 64)
            {
                colorMask[vertexIndex] |= (1UL << color);
                return;
            }

            List<int> overflow = overflowColors[vertexIndex];
            if (overflow == null)
            {
                overflow = new List<int>(4);
                overflowColors[vertexIndex] = overflow;
            }
            overflow.Add(color);
        }

        private static void AddEdge(int a, int b, HashSet<long> set, List<int> list, List<float> lens, Vector3[] pos)
        {
            int lo = Mathf.Min(a, b), hi = Mathf.Max(a, b);
            long key = (long)lo << 32 | (uint)hi;
            if (!set.Add(key)) return;
            list.Add(lo); list.Add(hi);
            lens.Add((pos[lo] - pos[hi]).magnitude);
        }

        private static void AddBendEdge(Dictionary<long, List<int>> edgeToOpposites, int a, int b, int opposite)
        {
            int lo = Mathf.Min(a, b), hi = Mathf.Max(a, b);
            long key = (long)lo << 32 | (uint)hi;

            List<int> opposites;
            if (!edgeToOpposites.TryGetValue(key, out opposites))
            {
                opposites = new List<int>(2);
                edgeToOpposites[key] = opposites;
            }

            opposites.Add(opposite);
        }

        private void BuildClothEntries()
        {
            entries.Clear();
            GameObject[] objClothes = chaCtrl.objClothes;
            if (objClothes != null)
            {
                for (int slot = 0; slot < objClothes.Length; slot++)
                {
                    string catId   = slot < ClothSlotNames.Length ? ClothSlotNames[slot] : "Slot" + slot;
                    GameObject go  = objClothes[slot];
                    if (go == null) continue;

                    var entry = new ClothPhysicsEntry
                    {
                        CategoryId  = catId,
                        DisplayName = catId,
                        IsExpanded  = false
                    };

                    foreach (var smr in go.GetComponentsInChildren<SkinnedMeshRenderer>(true))
                    {
                        if (smr == null || smr.sharedMesh == null) continue;
                        if (smr.sharedMesh.vertexCount < 30) continue; // skip trivial meshes
                        entry.Meshes.Add(new ClothMeshState
                        {
                            CategoryId = catId,
                            MeshName   = smr.name,
                            Renderer   = smr,
                            SkinBones  = smr.bones,
                        });
                            // Pre-compute which bones dominate ≥1 vertex so the UI can hide
                            // bones that wouldn't pin anything if selected.
                            var dominantSet = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
                            BoneWeight[] preWeights = smr.sharedMesh.boneWeights;
                            Transform[]  preBones   = smr.bones;
                            if (preWeights != null && preBones != null)
                            {
                                for (int pi = 0; pi < preWeights.Length; pi++)
                                {
                                    BoneWeight bw = preWeights[pi];
                                    int dom = bw.boneIndex0; float dw = bw.weight0;
                                    if (bw.weight1 > dw) { dw = bw.weight1; dom = bw.boneIndex1; }
                                    if (bw.weight2 > dw) { dw = bw.weight2; dom = bw.boneIndex2; }
                                    if (bw.weight3 > dw) { dom = bw.boneIndex3; }
                                    if (dom >= 0 && dom < preBones.Length && preBones[dom] != null)
                                        dominantSet.Add(preBones[dom].name);
                                }
                            }
                            entry.Meshes[entry.Meshes.Count - 1].BonesWithDominantVertices = dominantSet;
                    }

                    if (entry.Meshes.Count > 0)
                        entries.Add(entry);
                }
            }
        }
        public void RefreshEntries()
        {
            for (int i = activeStates.Count - 1; i >= 0; i--)
                DeactivateMesh(activeStates[i]);
            BuildClothEntries();
        }

        /// <summary>
        /// Completely stops cloth simulation for this character by deactivating
        /// all active cloth meshes and restoring their original renderers.
        /// </summary>
        public void DisableAllSimulation(bool clearSdfCollider = false)
        {
            for (int i = activeStates.Count - 1; i >= 0; i--)
                DeactivateMesh(activeStates[i]);

            if (clearSdfCollider)
                ClearSDFCollider();
        }

        private void ClearSDFCollider()
        {
            if (_sdfProxy != null)
            {
                Destroy(_sdfProxy);
                _sdfProxy = null;
            }
            SDFVoxelCount  = 0;
            SDFVertexCount = 0;
        }

        // ================================================================== //
        // SDF collider building
        // ================================================================== //

        private static readonly (string Name, string[] Patterns)[] SDFBoneGroupDefs =
        {
            ("Head",    new[] { "cf_j_head", "cf_j_neck", "cf_j_face" }),
            ("Torso",   new[] { "cf_j_spine", "cf_j_kosi", "cf_j_shoulder", "cf_j_siri", "cf_hit_siri", "cf_j_kokan", "cf_n_height" }),
            ("Arms",    new[] { "cf_j_armup", "cf_j_armlow", "cf_j_armelbo", "cf_j_hand", "cf_j_wrist" }),
            ("Legs",    new[] { "cf_j_legup", "cf_j_leglow", "cf_j_legknee", "cf_j_legupdam", "cf_j_foot", "cf_j_toes", "ankle" }),
            ("Breasts", new[] { "mune", "cf_hit_mune", "cf_j_mune_nip", "bust", "bnip" }),
        };

        public static readonly string[] SDFBoneGroupNames =
        {
            "Head", "Torso", "Arms", "Legs", "Breasts"
        };

        private static HashSet<string> GetBoneNamesForSDFGroup(Transform[] allBones, string groupName)
        {
            var result = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            string[] patterns = null;

            for (int i = 0; i < SDFBoneGroupDefs.Length; i++)
            {
                if (string.Equals(SDFBoneGroupDefs[i].Name, groupName, StringComparison.OrdinalIgnoreCase))
                {
                    patterns = SDFBoneGroupDefs[i].Patterns;
                    break;
                }
            }

            if (patterns == null || allBones == null) return result;

            foreach (Transform bone in allBones)
            {
                if (bone == null) continue;
                string lower = bone.name.ToLowerInvariant();
                for (int p = 0; p < patterns.Length; p++)
                {
                    if (lower.Contains(patterns[p]))
                    {
                        result.Add(bone.name);
                        break;
                    }
                }
            }

            return result;
        }

        /// <summary>
        /// SDF resolution quality level (1=low 16³, 2=medium 24³, 3=high 32³, 4=ultra 48³).
        /// </summary>
        public int SDFQuality { get; set; } = 3;

        /// <summary>
        /// Builds SDF body collider from selected body bone groups.
        /// Invoke this from the GUI whenever body proportions change.
        /// </summary>
        public void BuildSDFCollider(Dictionary<string, bool> sdfGroupEnabled)
        {
            SDFVoxelCount      = 0;
            SDFVertexCount     = 0;

            if (chaCtrl == null || sdfGroupEnabled == null) return;

            SkinnedMeshRenderer bodySMR = FindBodySMR();
            if (bodySMR == null) return;

            Transform[] allBones = bodySMR.bones;
            var sdfBoneNames     = new HashSet<string>(StringComparer.OrdinalIgnoreCase);

            foreach (var kv in sdfGroupEnabled)
            {
                if (!kv.Value) continue;
                var groupBones = GetBoneNamesForSDFGroup(allBones, kv.Key);
                foreach (var b in groupBones) sdfBoneNames.Add(b);
            }

            // ── SDF path: signed distance field ──
            if (sdfBoneNames.Count > 0)
            {
                // Map quality level to resolution
                int res;
                int decimation;
                float surfaceOffset;
                float gradientFactor;
                switch (SDFQuality)
                {
                    case 1:
                        res = 20;
                        decimation = 4;
                        surfaceOffset = 0.025f;
                        gradientFactor = 1.0f;
                        break;
                    case 2:
                        res = 28;
                        decimation = 3;
                        surfaceOffset = 0.035f;
                        gradientFactor = 1.2f;
                        break;
                    default:
                        res = 32;
                        decimation = 3;
                        surfaceOffset = 0.045f;
                        gradientFactor = 1.4f;
                        break;
                    case 4:
                        res = 48;
                        decimation = 2;
                        surfaceOffset = 0.05f;
                        gradientFactor = 1.5f;
                        break;
                }

                if (_sdfProxy == null) _sdfProxy = gameObject.AddComponent<SDFBodyCollider>();
                _sdfProxy.decimationFactor = decimation;
                _sdfProxy.resolution       = res;
                _sdfProxy.padding          = 0.06f;
                _sdfProxy.surfaceOffset    = surfaceOffset;
                _sdfProxy.gradientSampleFactor = gradientFactor;
                _sdfProxy.updateInterval   = 1;
                _sdfProxy.Initialize(bodySMR, sdfBoneNames);
                if (_sdfProxy.VertexCount <= 0) _sdfProxy.Initialize(bodySMR, null);

                // Build once immediately so counters are valid before the simulation is active.
                _sdfProxy.UpdateSDF();
                SDFVoxelCount   = _sdfProxy.VoxelCount;
                SDFVertexCount  = _sdfProxy.VertexCount;
            }
            else
            {
                ClearSDFCollider();
            }
        }

        /// <summary>Returns the body SkinnedMeshRenderer with the most vertices.</summary>
        private SkinnedMeshRenderer FindBodySMR()
        {
            if (chaCtrl.objBody == null) return null;
            SkinnedMeshRenderer best     = null;
            int                 bestVerts = 0;
            var smrs = chaCtrl.objBody.GetComponentsInChildren<SkinnedMeshRenderer>(true);
            foreach (var smr in smrs)
            {
                if (smr == null || smr.sharedMesh == null) continue;
                int v = smr.sharedMesh.vertexCount;
                if (v > bestVerts) { bestVerts = v; best = smr; }
            }
            return best;
        }
    }
}
