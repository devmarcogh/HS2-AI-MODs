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
    [DefaultExecutionOrder(10000)] // Run after IK/FK mods so bone transforms are final
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

        // ── Optimisation: cloth-to-cloth spatial hash grid ─────────────────────────
        private readonly SpatialHashGrid _clothGrid     = new SpatialHashGrid();
        private readonly List<int>       _clothQueryBuf = new List<int>(64);

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
            int n = s.VertCount;
            for (int i = 0; i < n; i++)
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

            // Allocate trigger-cooldown array for ManualDeformation mode
            EnsureTriggerCooldown(state);

            WriteMesh(state);
        }

        // ------------------------------------------------------------------ //
        // Triggered-simulation helpers
        // ------------------------------------------------------------------ //

        /// <summary>
        /// Returns the closest point on a capsule segment to world-space point p.
        /// Extracted from SolveVertexCollision so it can be reused in UpdateTriggerMasks.
        /// </summary>
        private static Vector3 GetCapsuleClosestPoint(Vector3 p, Vector3 p1, Vector3 v, float vDotV)
        {
            if (vDotV < 1e-10f) return p1;
            float t = Mathf.Clamp01(Vector3.Dot(p - p1, v) / vDotV);
            return p1 + t * v;
        }

        // ── Warm-up: allocate TriggerCooldown for ManualDeformation meshes ──
        private void EnsureTriggerCooldown(ClothMeshState state)
        {
            int n = state.VertCount;
            if (state.TriggerCooldown == null || state.TriggerCooldown.Length < n)
                state.TriggerCooldown = new int[n];
            else
                Array.Clear(state.TriggerCooldown, 0, n);
        }

        /// <summary>
        /// After all substeps, restore per-vertex InvMass for the next frame.
        /// Only called after triggered-mode substeps where InvMass was temporarily zeroed.
        /// </summary>
        private static void RestoreInvMass(ClothMeshState s)
        {
            for (int i = 0; i < s.VertCount; i++)
                if (!s.IsPinned[i] && s.Mass[i] > 0f)
                    s.InvMass[i] = 1f / s.Mass[i];
        }
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
            UpdateManualDeformationInput();

            float dt = Mathf.Clamp(Time.deltaTime, 0.001f, 0.05f);

            foreach (ClothMeshState state in activeStates)
            {
                if (state == null || state.Renderer == null) continue;

                // Per-mesh pause: skip simulation but keep the mesh rendered as-is.
                if (state.SimulationPaused) continue;

                // ManualDeformation: update per-vertex wake masks BEFORE bone-follow
                // and substeps.  Also temporarily zeros InvMass for sleeping vertices.
                if (state.SimulationMode == ClothSimulationMode.ManualDeformation)
                    UpdateManualDeformMasks(state);

                // LBS skinning + per-vertex bone-follow using per-mesh cached buffers.
                SkinAndFollowBones(state);

                // Substeps is a quality multiplier (0.25..1). Map to concrete count.
                int   substeps = Mathf.Max(1, Mathf.RoundToInt(state.Params.Substeps));
                float subDt    = dt / substeps;

                // 3-tier fallback: GPU → Native → Managed
                if (GPUClothAvailable)
                {
                    SimulateAllSubstepsGPU(state, dt, substeps);
                }
                else
                {
                    for (int sub = 0; sub < substeps; sub++)
                        SimulateStep(state, subDt);
                }

                // Restore per-vertex InvMass if manual-deform mode zeroed it
                if (state.SimulationMode == ClothSimulationMode.ManualDeformation)
                    RestoreInvMass(state);

                // Blend zone: lerp simulated positions toward LBS for partially-pinned vertices
                ApplyPinBlend(state);

                if (state.Params.ClothToCloth)
                    ApplyClothToClothCollision(state);

                WriteMesh(state);

                // Count down warmup
                if (state.WarmupFramesRemaining > 0)
                    state.WarmupFramesRemaining--;
            }
        }

        // ── Manual Deformation input (runs once per LateUpdate) ──────────────
        // Resolves the world-space cursor position by raycasting the camera ray
        // against the cloth mesh's AABB plane.  Stores the result in each
        // ManualDeformation mesh so UpdateManualDeformMasks can use it.
        private Vector3 _manualDeformCursorWorld;
        private bool    _manualDeformActive;

        /// <summary>True while the user holds the ManualDeform keybinding + RMB. Used by BaseUI to suppress camera control.</summary>
        internal static bool ManualDeformActive { get; private set; }

        private void UpdateManualDeformationInput()
        {
            // Check keybinding: modifier key(s) must be held AND right mouse button pressed.
            bool keyHeld = StudioCharaEditor.KeyManualDeform != null &&
                           IsManualDeformKeyHeld();
            bool rmb     = Input.GetMouseButton(1);
            _manualDeformActive  = keyHeld && rmb;
            ManualDeformActive   = _manualDeformActive;

            if (!_manualDeformActive)
            {
                // Clear drag state on all ManualDeform meshes so they auto-pause.
                for (int i = 0; i < activeStates.Count; i++)
                {
                    ClothMeshState s = activeStates[i];
                    if (s == null) continue;
                    if (s.SimulationMode == ClothSimulationMode.ManualDeformation)
                    {
                        s.IsDragging      = false;
                        s.DragVertexIndex = -1;
                        s.SimulationPaused = true;
                    }
                }
                return;
            }

            // Resolve cursor → world via camera ray against the Y=0 world plane
            // (approximation; good enough for cloth that sits near body height).
            Camera cam = Camera.main;
            if (cam == null) return;

            Ray ray = cam.ScreenPointToRay(Input.mousePosition);
            // Intersect against a horizontal plane at the average height of all active meshes.
            float planeY = 0f;
            int   cnt    = 0;
            for (int i = 0; i < activeStates.Count; i++)
            {
                ClothMeshState s = activeStates[i];
                if (s == null || s.SimulationMode != ClothSimulationMode.ManualDeformation) continue;
                planeY += s.WorldBounds.center.y;
                cnt++;
            }
            if (cnt > 0) planeY /= cnt;

            float denom = ray.direction.y;
            if (Mathf.Abs(denom) > 1e-6f)
            {
                float t = (planeY - ray.origin.y) / denom;
                if (t > 0f)
                    _manualDeformCursorWorld = ray.origin + ray.direction * t;
            }

            // Activate all ManualDeformation meshes while key+RMB held.
            for (int i = 0; i < activeStates.Count; i++)
            {
                ClothMeshState s = activeStates[i];
                if (s == null || s.SimulationMode != ClothSimulationMode.ManualDeformation) continue;
                s.SimulationPaused = false;
                s.IsDragging       = true;
                s.DragTargetWorld  = _manualDeformCursorWorld;

                // Find the closest vertex to the cursor each frame (cheap linear scan).
                float bestSq = float.MaxValue;
                int   bestI  = -1;
                for (int vi = 0; vi < s.VertCount; vi++)
                {
                    float sq = (s.Position[vi] - _manualDeformCursorWorld).sqrMagnitude;
                    if (sq < bestSq) { bestSq = sq; bestI = vi; }
                }
                s.DragVertexIndex = bestI;
            }
        }

        private static bool IsManualDeformKeyHeld()
        {
            KeyboardShortcut ks = StudioCharaEditor.KeyManualDeform.Value;
            if (!Input.GetKey(ks.MainKey)) return false;
            foreach (KeyCode mod in ks.Modifiers)
                if (!Input.GetKey(mod)) return false;
            return true;
        }

        /// <summary>
        /// For ManualDeformation meshes: decay cooldown, wake vertices within
        /// DeformRadius of the cursor (and attract the dragged vertex toward it),
        /// zero InvMass for sleeping vertices.
        /// </summary>
        private void UpdateManualDeformMasks(ClothMeshState s)
        {
            if (s.SimulationMode != ClothSimulationMode.ManualDeformation) return;
            if (s.TriggerCooldown == null || s.Mass == null) return;

            int   n   = s.VertCount;
            float rad = s.DeformRadius;

            bool   dragging  = s.IsDragging && s.DragVertexIndex >= 0;
            Vector3 cursor   = _manualDeformCursorWorld;
            float   radSq    = rad * rad;

            for (int i = 0; i < n; i++)
            {
                if (s.IsPinned[i]) continue;

                // Decay cooldown
                if (s.TriggerCooldown[i] > 0) s.TriggerCooldown[i]--;

                // Wake vertices within DeformRadius of the cursor
                if (dragging && (s.Position[i] - cursor).sqrMagnitude < radSq)
                    s.TriggerCooldown[i] = ClothMeshState.TriggerCooldownFrames;

                // Apply spring attraction to the dragged vertex and its neighbours
                if (dragging && i == s.DragVertexIndex && s.TriggerCooldown[i] > 0)
                {
                    // Direct impulse: pull vertex toward cursor each frame
                    Vector3 delta = cursor - s.Position[i];
                    s.Velocity[i] += delta * 12f * Time.deltaTime; // spring constant
                }

                if (s.TriggerCooldown[i] == 0)
                {
                    s.InvMass[i]  = 0f;
                    s.Velocity[i] = Vector3.zero;
                }
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

        private void SimulateStep(ClothMeshState s, float dt)
        {
            int n = s.VertCount;

            Array.Copy(s.Position, s.PrevPosition, n);

            // ── Native fast path ──
            if (NativeAvailable)
            {
                SimulateStepNative(s, dt);
                return;
            }

            // 1. Predict (parallel when large enough)
            // Note: triggered-frozen verts have InvMass=0 (set by UpdateTriggerMasks).
            // Checking InvMass<=0 here covers both bone-pinned AND triggered-frozen verts.
            float g = s.Params.Gravity * s.Params.Weight;
            if (n > 128)
            {
                Vector3[] pos  = s.Position;
                Vector3[] pred = s.PredPosition;
                Vector3[] vel  = s.Velocity;
                float[]   inv  = s.InvMass;
                Parallel.For(0, n, i =>
                {
                    if (inv[i] <= 0f) { pred[i] = pos[i]; return; }
                    Vector3 v = vel[i]; v.y += g * dt; vel[i] = v;
                    pred[i] = pos[i] + v * dt;
                });
            }
            else
            {
                for (int i = 0; i < n; i++)
                {
                    if (s.InvMass[i] <= 0f) { s.PredPosition[i] = s.Position[i]; continue; }
                    s.Velocity[i].y += g * dt;
                    s.PredPosition[i] = s.Position[i] + s.Velocity[i] * dt;
                }
            }

            // 3. Constraints Iterations
            float tildedCompliance = 1f / Mathf.Max(1e-6f, s.Params.StretchStiffness * dt * dt);
            float bendCompliance   = 1f / Mathf.Max(1e-6f, s.Params.BendStiffness * dt * dt);
            float thick = s.Params.Thickness;

            // Snapshot collider array reference + count for thread safety (immutable during iteration)
            MirrorCollider[] colArr = _mirrorColArr;
            int              colCnt = _mirrorColCount;

            // SDF collision state
            bool useSDF = UseSDFColliders && _sdfProxy != null && _sdfProxy.IsReady;
            SDFBodyCollider sdfCol = useSDF ? _sdfProxy : null;

            int iterations = Mathf.Max(1, Mathf.RoundToInt(s.Params.Iterations));
            for (int iter = 0; iter < iterations; iter++)
            {
                SolveEdges_XPBD(s, tildedCompliance);
                if (s.BendPairs != null && s.BendPairs.Length > 0)
                    SolveBends_XPBD(s, bendCompliance);
            }

            // Collision pass — ONCE per substep, after all constraint iterations.
            // Skip frozen verts (InvMass=0 covers both bone-pinned and triggered-frozen).
            if (n > 256)
            {
                float[] invC = s.InvMass;
                Parallel.For(0, n, i =>
                {
                    if (invC[i] <= 0f) return;
                    s.PredPosition[i] = SolveVertexCollision(
                        s.PredPosition[i], colArr, colCnt, thick,
                        useSDF, sdfCol);
                });
            }
            else
            {
                for (int i = 0; i < n; i++)
                {
                    if (s.InvMass[i] <= 0f) continue;
                    s.PredPosition[i] = SolveVertexCollision(
                        s.PredPosition[i], colArr, colCnt, thick,
                        useSDF, sdfCol);
                }
            }

            // 4. Commit (parallel when large enough)
            float inv_dt   = 1f / dt;
            // During warmup, use extra-strong damping to let the cloth settle without
            // explosion from sudden collider contact on the first frames.
            float dampBase  = s.Params.Damping;
            if (s.WarmupFramesRemaining > 0)
                dampBase = Mathf.Lerp(dampBase, 30f,
                    s.WarmupFramesRemaining / (float)ClothMeshState.WarmupDuration);
            float damp     = Mathf.Clamp01(1f - dampBase * dt);
            // During warmup, cap velocity even tighter.
            float maxSpeed   = s.WarmupFramesRemaining > 0 ? 2f : 15f;
            float maxSpeedSq = maxSpeed * maxSpeed;
            // SDF surface friction: near the surface, cancel velocity pointing into the body.
            // Prevents jitter from voxel-edge noise injecting chaotic momentum.
            float frictionZone = thick * 2.5f;
            if (n > 128)
            {
                Vector3[] pos  = s.Position;
                Vector3[] pred = s.PredPosition;
                Vector3[] vel  = s.Velocity;
                float[]   invK = s.InvMass;
                Parallel.For(0, n, i =>
                {
                    if (invK[i] <= 0f) return;  // pinned or triggered-frozen
                    Vector3 v = (pred[i] - pos[i]) * inv_dt * damp;
                    if (v.sqrMagnitude > maxSpeedSq) v = v.normalized * maxSpeed;
                    v = StripInwardVelocity(v, pred[i], useSDF, sdfCol, frictionZone);
                    vel[i] = v;
                    pos[i] = pred[i];
                });
            }
            else
            {
                for (int i = 0; i < n; i++)
                {
                    if (s.InvMass[i] <= 0f) continue;  // pinned or triggered-frozen
                    Vector3 v = (s.PredPosition[i] - s.Position[i]) * inv_dt * damp;
                    if (v.sqrMagnitude > maxSpeedSq) v = v.normalized * maxSpeed;
                    v = StripInwardVelocity(v, s.PredPosition[i], useSDF, sdfCol, frictionZone);
                    s.Velocity[i] = v;
                    s.Position[i] = s.PredPosition[i];
                }
            }
        }

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
                n,
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
        }

        // ================================================================== //
        // Native xPBD substep — delegates hot loops to StudioModsNative.dll
        // ================================================================== //
        private void SimulateStepNative(ClothMeshState s, float dt)
        {
            int n = s.VertCount;
            EnsureNativeBuffers(s);
            SyncToNative(s);

            // 1. Predict
            float g = s.Params.Gravity * s.Params.Weight;
            NativeBridge.Cloth_Predict(_nPos, _nVel, _nPred, _nInvMass, n, g, dt);

            // 2. Constraint iterations
            float tildedCompliance = 1f / Mathf.Max(1e-6f, s.Params.StretchStiffness * dt * dt);
            float bendCompliance   = 1f / Mathf.Max(1e-6f, s.Params.BendStiffness * dt * dt);
            float thick = s.Params.Thickness;

            int iterations = Mathf.Max(1, Mathf.RoundToInt(s.Params.Iterations));
            for (int iter = 0; iter < iterations; iter++)
            {
                // Solve edges per color group (sequential between groups for correctness)
                if (s.EdgeColorGroups != null)
                {
                    float pressScale = 1f - s.Params.Compression * MaxCompressionRatio;
                    foreach (int[] group in s.EdgeColorGroups)
                    {
                        int[] groupEdges = new int[group.Length * 2];
                        float[] groupRestLen = new float[group.Length];
                        for (int gi = 0; gi < group.Length; gi++)
                        {
                            int eIdx = group[gi];
                            groupEdges[gi * 2]     = s.Edges[eIdx * 2];
                            groupEdges[gi * 2 + 1] = s.Edges[eIdx * 2 + 1];
                            groupRestLen[gi]        = s.RestEdgeLen[eIdx];
                        }
                        NativeBridge.Cloth_SolveEdges(_nPred, _nInvMass, _nIsPinned,
                            groupEdges, groupRestLen, group.Length,
                            tildedCompliance, pressScale);
                    }
                }
                else
                {
                    float[] restLen = s.RestEdgeLen;
                    float pressScale = 1f - s.Params.Compression * MaxCompressionRatio;
                    NativeBridge.Cloth_SolveEdges(_nPred, _nInvMass, _nIsPinned,
                        s.Edges, restLen, s.Edges.Length / 2,
                        tildedCompliance, pressScale);
                }

                // Solve bends per color group
                if (s.BendPairs != null && s.BendPairs.Length > 0)
                {
                    if (s.BendColorGroups != null)
                    {
                        foreach (int[] group in s.BendColorGroups)
                        {
                            int[] groupBends = new int[group.Length * 2];
                            float[] groupRestBendLen = new float[group.Length];
                            for (int gi = 0; gi < group.Length; gi++)
                            {
                                int bIdx = group[gi];
                                groupBends[gi * 2]      = s.BendPairs[bIdx * 2];
                                groupBends[gi * 2 + 1]  = s.BendPairs[bIdx * 2 + 1];
                                groupRestBendLen[gi]     = s.RestBendLen[bIdx];
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
            }

            // 3. Collision pass — capsule/sphere colliders
            BuildNativeColliders();
            if (_mirrorColCount > 0)
                NativeBridge.Cloth_SolveCollision(_nPred, _nInvMass, n, _nColliders, _mirrorColCount, thick);

            // SDF collision pass (stays managed for now — SDF data lives in SDFBodyCollider)
            bool useSDF = UseSDFColliders && _sdfProxy != null && _sdfProxy.IsReady;
            if (useSDF)
            {
                // Write pred back to managed for SDF, then re-read
                NativeBridge.FlatToVec3Array(_nPred, s.PredPosition, n);
                for (int i = 0; i < n; i++)
                {
                    if (s.InvMass[i] <= 0f) continue;
                    s.PredPosition[i] = SolveVertexCollision(
                        s.PredPosition[i], _mirrorColArr, 0, thick, true, _sdfProxy);
                }
                for (int i = 0; i < n; i++)
                {
                    int i3 = i * 3;
                    _nPred[i3] = s.PredPosition[i].x;
                    _nPred[i3 + 1] = s.PredPosition[i].y;
                    _nPred[i3 + 2] = s.PredPosition[i].z;
                }
            }

            // 4. Commit
            float inv_dt = 1f / dt;
            float dampBase = s.Params.Damping;
            if (s.WarmupFramesRemaining > 0)
                dampBase = Mathf.Lerp(dampBase, 30f,
                    s.WarmupFramesRemaining / (float)ClothMeshState.WarmupDuration);
            float damp     = Mathf.Clamp01(1f - dampBase * dt);
            float maxSpeed = s.WarmupFramesRemaining > 0 ? 2f : 15f;

            NativeBridge.Cloth_Commit(_nPos, _nVel, _nPred, _nInvMass, n, inv_dt, damp, maxSpeed);

            // Sync back to managed arrays
            SyncFromNative(s);
        }

        /// <summary>
        /// Pure function: resolve collisions and magnetic interactions for a single vertex.
        /// Thread-safe — no shared mutable state.
        /// MagneticMode: 0=Repel (normal push-out), 1=Attract (pull toward surface), 2=Off.
        /// </summary>
        private static Vector3 SolveVertexCollision(
            Vector3 p, MirrorCollider[] colArr, int colCnt, float thick,
            bool useSDF, SDFBodyCollider sdfCol)
        {
            // Mirror / proxy collider pass
            for (int c = 0; c < colCnt; c++)
            {
                MirrorCollider col = colArr[c];

                // Closest point on the collider primitive
                Vector3 closest;
                if (!col.IsCapsule)
                {
                    closest = col.Center;
                }
                else
                {
                    Vector3 w = p - col.P1;
                    float t = Mathf.Clamp01(Vector3.Dot(w, col.V) / col.VDotV);
                    closest = col.P1 + t * col.V;
                }

                Vector3 delta  = p - closest;
                float   distSq = delta.sqrMagnitude;
                float   dist   = Mathf.Sqrt(distSq);
                float   minSep = col.Radius + thick;
                Vector3 normal = (dist > 1e-6f) ? (delta / dist) : Vector3.up;

                switch (col.MagneticMode)
                {
                    case 1: // Attract — pull cloth toward (and cling to) the collider surface.
                    {
                        // Any vertex within MagneticRange is snapped toward the surface.
                        // MagneticStrength (0→1) is the per-substep blend fraction:
                        //   1.0 = instant full snap, 0.5 = gentle pull over several frames.
                        // Vertices that have already penetrated are always fully pushed out.
                        float range = col.MagneticRange;
                        if (dist < range)
                        {
                            Vector3 onSurface = closest + normal * minSep;
                            float   blend     = dist < minSep
                                ? 1f                                       // inside: full push-out
                                : Mathf.Clamp01(col.MagneticStrength);    // outside: strength-controlled pull
                            p = Vector3.Lerp(p, onSurface, blend);
                        }
                        break;
                    }
                    case 0: // Repel — default collision (push out when penetrating)
                    default:
                    {
                        if (distSq < minSep * minSep)
                            p = closest + normal * minSep;
                        break;
                    }
                    // case 2 (Off): skipped entirely by UpdateMirrorColliders
                }
            }

            // SDF collision — O(1) trilinear interpolation + gradient push
            if (useSDF && sdfCol != null)
            {
                Unity.Mathematics.float3 fp = new Unity.Mathematics.float3(p.x, p.y, p.z);
                float dist = sdfCol.SampleDistance(fp);
                if (dist < thick)
                {
                    Unity.Mathematics.float3 grad = sdfCol.SampleGradient(fp);
                    float gradLen = Unity.Mathematics.math.length(grad);
                    float pushAmount = thick - dist;

                    if (gradLen > 1e-6f)
                    {
                        Unity.Mathematics.float3 normal = grad / gradLen;
                        Unity.Mathematics.float3 push = normal * pushAmount;
                        p.x += push.x;
                        p.y += push.y;
                        p.z += push.z;
                    }
                    else
                    {
                        p.y += pushAmount; // fallback: push up
                    }
                }
            }

            return p;
        }

        /// <summary>
        /// Removes the velocity component pointing into the SDF body surface.
        /// Near the surface, voxel-edge noise injects chaotic push directions each frame;
        /// letting those push vectors become velocity causes bounce/jitter.
        /// Cancelling the inward-normal component keeps tangential sliding while
        /// preventing energy injection from the collision response.
        /// Thread-safe — only reads the SDF.
        /// </summary>
        private static Vector3 StripInwardVelocity(
            Vector3 v, Vector3 pos,
            bool useSDF, SDFBodyCollider sdfCol, float frictionZone)
        {
            if (!useSDF || sdfCol == null) return v;

            Unity.Mathematics.float3 fp = new Unity.Mathematics.float3(pos.x, pos.y, pos.z);
            float dist = sdfCol.SampleDistance(fp);

            // Only apply friction near the surface (inside the friction zone)
            if (dist >= frictionZone) return v;

            Unity.Mathematics.float3 grad = sdfCol.SampleGradient(fp);
            float gradLen = Unity.Mathematics.math.length(grad);
            if (gradLen < 1e-6f) return v;

            // Surface normal (pointing outward from body)
            Unity.Mathematics.float3 normal = grad / gradLen;
            Vector3 n = new Vector3(normal.x, normal.y, normal.z);

            // Project velocity onto normal
            float vn = Vector3.Dot(v, n);

            // Only strip the component pointing INTO the surface (vn < 0)
            if (vn >= 0f) return v;

            // Blend: full stripping at surface, fading to zero at frictionZone
            float blend = 1f - Mathf.Clamp01(dist / frictionZone);
            return v - n * (vn * blend);
        }

        // MaxCompressionRatio: at Compression=1 edges target 65% of rest length.
        // Keeping it below 50% avoids degeneracy while giving meaningful press-against-body effect.

        private static void SolveEdges_XPBD(ClothMeshState s, float tildedCompliance)
        {
            float pressScale = 1f - s.Params.Compression * MaxCompressionRatio;

            // Use graph-color groups when available (enables safe Parallel.For per color)
            if (s.EdgeColorGroups != null && s.EdgeColorGroups.Length > 0)
            {
                for (int c = 0; c < s.EdgeColorGroups.Length; c++)
                {
                    int[] group = s.EdgeColorGroups[c];
                    // Each group contains flat [pairIndex, pairIndex, ...] indices into Edges/RestEdgeLen
                    if (group.Length > 64)
                    {
                        Parallel.For(0, group.Length, gi =>
                            SolveEdgePair(s, group[gi], tildedCompliance, pressScale));
                    }
                    else
                    {
                        for (int gi = 0; gi < group.Length; gi++)
                            SolveEdgePair(s, group[gi], tildedCompliance, pressScale);
                    }
                }
            }
            else
            {
                // Fallback: sequential (no color groups built yet)
                for (int e = 0; e < s.Edges.Length; e += 2)
                    SolveEdgePairDirect(s, e, tildedCompliance, pressScale);
            }
        }

        private static void SolveEdgePair(ClothMeshState s, int pairIdx, float tc, float ps)
            => SolveEdgePairDirect(s, pairIdx * 2, tc, ps);

        private static void SolveEdgePairDirect(ClothMeshState s, int e, float tildedCompliance, float pressScale)
        {
            int i = s.Edges[e], j = s.Edges[e + 1];
            float wi = s.IsPinned[i] ? 0f : s.InvMass[i];
            float wj = s.IsPinned[j] ? 0f : s.InvMass[j];
            if (wi == 0f && wj == 0f) return;
            float wSum = wi + wj + tildedCompliance;
            if (wSum < 1e-10f) return;
            Vector3 d   = s.PredPosition[i] - s.PredPosition[j];
            float   len = d.magnitude;
            if (len < 1e-7f) return;
            float   C      = len - s.RestEdgeLen[e / 2] * pressScale;
            float   lambda = -C / wSum;
            Vector3 corr   = (d / len) * lambda;
            if (!s.IsPinned[i]) s.PredPosition[i] += wi * corr;
            if (!s.IsPinned[j]) s.PredPosition[j] -= wj * corr;
        }

        private static void SolveBends_XPBD(ClothMeshState s, float tildedCompliance)
        {
            if (s.BendColorGroups != null && s.BendColorGroups.Length > 0)
            {
                for (int c = 0; c < s.BendColorGroups.Length; c++)
                {
                    int[] group = s.BendColorGroups[c];
                    if (group.Length > 64)
                    {
                        Parallel.For(0, group.Length, gi =>
                            SolveBendPairDirect(s, group[gi] * 2, tildedCompliance));
                    }
                    else
                    {
                        for (int gi = 0; gi < group.Length; gi++)
                            SolveBendPairDirect(s, group[gi] * 2, tildedCompliance);
                    }
                }
            }
            else
            {
                for (int b = 0; b < s.BendPairs.Length; b += 2)
                    SolveBendPairDirect(s, b, tildedCompliance);
            }
        }

        private static void SolveBendPairDirect(ClothMeshState s, int b, float tildedCompliance)
        {
            int i = s.BendPairs[b], j = s.BendPairs[b + 1];
            float wi = s.IsPinned[i] ? 0f : s.InvMass[i];
            float wj = s.IsPinned[j] ? 0f : s.InvMass[j];
            if (wi == 0f && wj == 0f) return;
            float wSum = wi + wj + tildedCompliance;
            if (wSum < 1e-10f) return;
            Vector3 d   = s.PredPosition[i] - s.PredPosition[j];
            float   len = d.magnitude;
            if (len < 1e-7f) return;
            float   C      = len - s.RestBendLen[b / 2];
            float   lambda = -C / wSum;
            Vector3 corr   = (d / len) * lambda;
            if (!s.IsPinned[i]) s.PredPosition[i] += wi * corr;
            if (!s.IsPinned[j]) s.PredPosition[j] -= wj * corr;
        }

        private void ApplyClothToClothCollision(ClothMeshState s)
        {
            float twoT   = s.Params.Thickness * 2f;
            float twoTSq = twoT * twoT;

            foreach (ClothMeshState other in activeStates)
            {
                if (other == s || !other.Params.ClothToCloth) continue;
                if (!s.WorldBounds.Intersects(other.WorldBounds)) continue;

                // Build spatial hash from the other mesh's positions for O(n+m) collision
                _clothGrid.Build(other.Position, twoT);

                for (int i = 0; i < s.VertCount; i++)
                {
                    if (s.IsPinned[i]) continue;
                    Vector3 pi = s.Position[i];

                    _clothGrid.Query(pi, twoT, _clothQueryBuf);
                    for (int qi = 0; qi < _clothQueryBuf.Count; qi++)
                    {
                        int j = _clothQueryBuf[qi];
                        if (other.IsPinned[j]) continue;
                        Vector3 d   = pi - other.Position[j];
                        float   dSq = d.sqrMagnitude;
                        if (dSq >= twoTSq || dSq < 1e-10f) continue;

                        float   dist  = Mathf.Sqrt(dSq);
                        float   pen   = twoT - dist;
                        Vector3 n     = d / dist;
                        Vector3 push  = n * (pen * 0.5f);

                        s.Position[i]     += push;
                        other.Position[j] -= push;
                    }
                }
            }
        }
        private void WriteMesh(ClothMeshState s)
        {
            if (s.WorkMesh == null || s.Filter == null || s.LocalVerts == null) return;

            // Cache worldToLocalMatrix to avoid per-vertex virtual call overhead
            Transform tf  = s.Renderer != null ? s.Renderer.transform : transform;
            Matrix4x4 w2l = tf.worldToLocalMatrix;
            int       vc  = s.VertCount;
            if (vc > 256)
            {
                Vector3[] pos  = s.Position;
                Vector3[] lv   = s.LocalVerts;
                Parallel.For(0, vc, i => { lv[i] = w2l.MultiplyPoint3x4(pos[i]); });
            }
            else
            {
                for (int i = 0; i < vc; i++)
                    s.LocalVerts[i] = w2l.MultiplyPoint3x4(s.Position[i]);
            }

            s.WorkMesh.vertices = s.LocalVerts;
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
        /// Uses per-mesh SkinnedPositions / PrevSkinnedPositions buffers so that
        /// multiple active cloth meshes cannot corrupt each other's bone-follow data.
        /// 1. Compute LBS skinned positions for every vertex from current bone transforms.
        /// 2. For non-pinned verts: shift Position/PrevPosition by the per-vertex delta
        ///    between this frame's LBS and last frame's LBS, capturing IK/FK rotations
        ///    and all bone motion.
        /// 3. For fully-pinned verts: set Position directly to LBS output.
        /// </summary>
        private void SkinAndFollowBones(ClothMeshState s)
        {
            if (s.SrcBindVerts == null || s.BindPoses == null ||
                s.SkinBones == null || s.VertexBoneWeights == null) return;

            int n = s.VertCount;
            int boneCount = s.SkinBones.Length;

            // Ensure per-mesh LBS caches are allocated
            if (s.SkinnedPositions == null || s.SkinnedPositions.Length < n)
                s.SkinnedPositions = new Vector3[n];
            if (s.PrevSkinnedPositions == null || s.PrevSkinnedPositions.Length < n)
                s.PrevSkinnedPositions = new Vector3[n];

            bool hasPrev = true; // buffers are always primed by WarmUpClothState

            // Build bone matrices once per frame (shared scratch buffer is safe here
            // because SkinAndFollowBones is called sequentially per mesh in LateUpdate)
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

                if (s.IsPinned[i])
                {
                    // Fully pinned: set position directly to LBS
                    s.Position[i]     = skinned;
                    s.PredPosition[i] = skinned;
                    s.Velocity[i]     = Vector3.zero;
                }
                else if (hasPrev && s.InvMass[i] > 0f)
                {
                    // Per-vertex bone-follow: shift by the delta between frames.
                    // Skip when InvMass=0 (triggered-frozen) — those verts stay frozen.
                    Vector3 delta = skinned - prev[i];
                    if (delta.sqrMagnitude > 1e-12f)
                    {
                        s.Position[i]     += delta;
                        s.PrevPosition[i] += delta;
                    }
                }
                // else: triggered-frozen (InvMass=0, not IsPinned) — position intentionally unchanged.
            }

            // Rotate buffers: current frame becomes previous for next frame
            Array.Copy(curr, prev, n);
        }

        /// <summary>
        /// After simulation: blend partially-pinned vertices toward their LBS position.
        /// Fully-pinned verts already set by SkinAndFollowBones; fully-simulated verts untouched.
        /// </summary>
        private void ApplyPinBlend(ClothMeshState s)
        {
            if (s.SkinnedPositions == null || s.PinBlend == null) return;
            for (int i = 0; i < s.VertCount; i++)
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
            // This compensates for RestInflate, sim drift, and any BakeMesh/LBS mismatch.
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

            // Clone rest-pose mesh (BakeMesh gives skinned local-space positions)
            Mesh baked = new Mesh();
            smr.BakeMesh(baked);
            baked.name = src.name + "_ClothPhysics";

            state.WorkMesh = baked;
            state.VertCount = baked.vertices.Length;
            state.WorkMesh.MarkDynamic(); // tells Unity this mesh is updated every frame → faster GPU upload
            int originalVertCount = state.VertCount;

            Transform tf       = smr.transform;
            Vector3[] bakedV   = baked.vertices;
            int[]     tris     = baked.triangles;
            state.Tris = tris;

            Vector3[] worldRest = new Vector3[state.VertCount];
            for (int i = 0; i < state.VertCount; i++)
                worldRest[i] = tf.TransformPoint(bakedV[i]);
            state.Position     = new Vector3[state.VertCount];
            state.Velocity     = new Vector3[state.VertCount];
            state.PredPosition = new Vector3[state.VertCount];
            state.PrevPosition = new Vector3[state.VertCount];
            state.IsPinned     = new bool[state.VertCount];
            state.Mass         = new float[state.VertCount];
            state.InvMass      = new float[state.VertCount];

            Array.Copy(worldRest, state.Position, state.VertCount);
            int[] newToOriginalMap;
            int[] vertexMapping = WeldVertices(state.Position, tris, 0.0001f);
            int weldedCount = 0;

            
            for (int i = 0; i < vertexMapping.Length; i++)
                if (vertexMapping[i] == i) weldedCount++;


            Vector2[] originalUvs = src.uv;
            Vector2[] weldedUvs = new Vector2[weldedCount];


            if (weldedCount < state.VertCount)
            {
                // Build a separate mapping: original index → final compacted index
                int[] originalToNewMap = new int[state.VertCount];
                int idx = 0;
                for (int i = 0; i < state.VertCount; i++)
                {
                    if (vertexMapping[i] == i){
                        if (originalUvs != null && i < originalUvs.Length)
                            weldedUvs[idx] = originalUvs[i];
                        originalToNewMap[i] = idx++;  // kept vertex gets new index
                    }
                    else
                        originalToNewMap[i] = -1;  // merged away
                }

                // Apply vertexMapping transitively to handle multi-level merges
                for (int i = 0; i < vertexMapping.Length; i++)
                {
                    if (vertexMapping[i] != i)
                    {
                        int rep = vertexMapping[i];
                        while (vertexMapping[rep] != rep)
                            rep = vertexMapping[rep];
                        vertexMapping[i] = rep;
                    }
                }
                Vector3[] weldedPos = new Vector3[weldedCount];
                bool[]    weldedPin = new bool[weldedCount];
                newToOriginalMap = new int[weldedCount];
                idx = 0;
                for (int i = 0; i < state.VertCount; i++)
                {
                    if (vertexMapping[i] == i)
                    {
                        weldedPos[idx]   = state.Position[i];
                        weldedPin[idx]   = state.IsPinned[i];
                        newToOriginalMap[idx] = i;
                        idx++;
                    }
                }

                state.Position  = weldedPos;
                state.IsPinned  = weldedPin;
                state.VertCount = weldedCount;
                worldRest       = weldedPos;

                for (int i = 0; i < tris.Length; i++)
                {
                    int oldIdx = tris[i];
                    int repIdx = vertexMapping[oldIdx];  // find representative
                    tris[i] = originalToNewMap[repIdx];  // convert to new compacted index
                }

                tris = FilterValidTriangles(tris, weldedCount);

                state.Tris = tris;

                state.Velocity     = new Vector3[weldedCount];
                state.PredPosition = new Vector3[weldedCount];
                state.PrevPosition = new Vector3[weldedCount];
                state.Mass         = new float[weldedCount];
                state.InvMass      = new float[weldedCount];
            }
            else
            {
                newToOriginalMap = new int[originalVertCount];
                for (int i = 0; i < originalVertCount; i++)
                    newToOriginalMap[i] = i;
                tris = FilterValidTriangles(tris, state.VertCount);
                state.Tris = tris;
            }

            state.WorkMesh.Clear();

            // ── Set up bone weights and skin bones BEFORE building WorkMesh ──
            // We need these to re-skin positions via manual LBS, since BakeMesh may
            // return bind/stale pose if the SMR hasn't been updated this frame.
            BoneWeight[] srcBoneWeights = src.boneWeights;
            state.VertexBoneWeights = new BoneWeight[state.VertCount];
            for (int i = 0; i < state.VertCount; i++)
            {
                int srcIdx = newToOriginalMap[i];
                if (srcBoneWeights != null && srcIdx >= 0 && srcIdx < srcBoneWeights.Length)
                    state.VertexBoneWeights[i] = srcBoneWeights[srcIdx];
                else
                    state.VertexBoneWeights[i] = default(BoneWeight);
            }
            state.SkinBones = smr.bones;

            // ── Store bind-pose data for per-frame LBS skinning of pinned vertices ──
            Matrix4x4[] bindPoses = src.bindposes;
            Vector3[]   srcVerts  = src.vertices;
            state.BindPoses = bindPoses;
            state.SrcBindVerts = new Vector3[state.VertCount];
            for (int i = 0; i < state.VertCount; i++)
            {
                int origIdx = newToOriginalMap[i];
                if (srcVerts != null && origIdx >= 0 && origIdx < srcVerts.Length)
                    state.SrcBindVerts[i] = srcVerts[origIdx];
            }

            // ── Manual LBS: guarantee state.Position matches current animation pose ──
            if (bindPoses != null && srcVerts != null && state.SkinBones != null && state.SkinBones.Length > 0)
            {
                Matrix4x4[] skinMats = new Matrix4x4[state.SkinBones.Length];
                for (int b = 0; b < state.SkinBones.Length; b++)
                {
                    skinMats[b] = (state.SkinBones[b] != null && b < bindPoses.Length)
                        ? state.SkinBones[b].localToWorldMatrix * bindPoses[b]
                        : Matrix4x4.identity;
                }

                for (int i = 0; i < state.VertCount; i++)
                {
                    int origIdx = newToOriginalMap[i];
                    if (origIdx < 0 || origIdx >= srcVerts.Length) continue;
                    Vector3 v = srcVerts[origIdx];
                    BoneWeight bw = state.VertexBoneWeights[i];

                    state.Position[i] =
                        (Vector3)skinMats[bw.boneIndex0].MultiplyPoint3x4(v) * bw.weight0 +
                        (Vector3)skinMats[bw.boneIndex1].MultiplyPoint3x4(v) * bw.weight1 +
                        (Vector3)skinMats[bw.boneIndex2].MultiplyPoint3x4(v) * bw.weight2 +
                        (Vector3)skinMats[bw.boneIndex3].MultiplyPoint3x4(v) * bw.weight3;
                }
                // worldRest may alias state.Position (welded path) — sync if separate
                if (!ReferenceEquals(worldRest, state.Position))
                    Array.Copy(state.Position, worldRest, state.VertCount);
            }

            Vector3[] localRestFinal = new Vector3[state.VertCount];
            for (int i = 0; i < state.VertCount; i++)
                localRestFinal[i] = tf.InverseTransformPoint(worldRest[i]);
            state.WorkMesh.vertices = localRestFinal;
            state.WorkMesh.triangles = tris;
            state.WorkMesh.uv = weldedUvs;
            state.WorkMesh.RecalculateBounds();
            state.WorkMesh.RecalculateNormals();

            // ── Rest Inflate: push vertices out along normals so cloth starts with volume ──
            float inflate = state.Params.RestInflate;
            if (inflate > 0.0001f)
            {
                Vector3[] meshNormals = state.WorkMesh.normals; // local-space normals (RecalculateNormals ran above)
                for (int i = 0; i < state.VertCount; i++)
                {
                    Vector3 worldNormal  = tf.TransformDirection(meshNormals[i]);
                    worldRest[i]        += worldNormal * inflate;
                    state.Position[i]    = worldRest[i];
                }
                // Rebuild WorkMesh vertices from inflated positions so rest lengths are correct
                for (int i = 0; i < state.VertCount; i++)
                    localRestFinal[i] = tf.InverseTransformPoint(worldRest[i]);
                state.WorkMesh.vertices = localRestFinal;
                state.WorkMesh.RecalculateBounds();
                state.WorkMesh.RecalculateNormals();
            }

            // Store rest positions in character-root local space (Compression + pinning).
            if (chaCtrl != null)
            {
                state.RestBodyLocalPos = new Vector3[state.VertCount];
                for (int i = 0; i < state.VertCount; i++)
                    state.RestBodyLocalPos[i] = chaCtrl.transform.InverseTransformPoint(worldRest[i]);
            }

            // Apply current selected-bone pinning.
            RecomputePinsFromSelectedBone(state);

            BuildEdges(state, tris, worldRest);
            BuildBends(state, tris, worldRest);
            // Mass and IsPinned are already set by RecomputePinsFromSelectedBone above.

            GameObject go = smr.gameObject;
            state.OriginalMaterials = smr.sharedMaterials;  // shared: no extra material instances
            smr.enabled = false;

            if (state.Filter == null)  state.Filter  = go.AddComponent<MeshFilter>();
            if (state.MeshRend == null) state.MeshRend = go.AddComponent<MeshRenderer>();
            if (state.ClothCollider == null)
                state.ClothCollider = go.GetComponent<MeshCollider>() ?? go.AddComponent<MeshCollider>();

            state.Filter.sharedMesh        = state.WorkMesh;
            state.MeshRend.sharedMaterials = state.OriginalMaterials;
            state.MeshRend.shadowCastingMode       = state.Renderer.shadowCastingMode;
            state.MeshRend.receiveShadows          = state.Renderer.receiveShadows;
            state.MeshRend.lightProbeUsage         = state.Renderer.lightProbeUsage;
            state.MeshRend.reflectionProbeUsage    = state.Renderer.reflectionProbeUsage;
            state.MeshRend.probeAnchor             = state.Renderer.probeAnchor;
            state.MeshRend.allowOcclusionWhenDynamic = state.Renderer.allowOcclusionWhenDynamic;

            state.ClothCollider.convex = false;
            state.ClothCollider.sharedMesh = null;
            state.ClothCollider.sharedMesh = state.WorkMesh;
            state.ClothCollider.enabled = true;

            state.LocalVerts = new Vector3[state.VertCount];
            // Initial pin positions will be set by SkinPinnedVertices on next frame
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
