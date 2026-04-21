using System;
using System.IO;
using System.Reflection;
using UnityEngine;

namespace StudioModsMSG
{

    // TODO: Revisar por que al pausar la simulacion sigue consumiendo CPU
    // TODO Aplicar tecnica de "sleep" para reducir el consumo de CPU cuando la simulacion esta pausada
    // TODO Aplicar tecnica smoothin en el SDF para reducir el jitter en colisiones rapidas (ej. pinball) en codigo c++ al igual que el solver XPBD, y comparar resultados con la version GPU para ver si el jitter se reduce sin perder estabilidad
    // TODO Revisar por que la simulacion no funciona en GPU
    // TDOO Mejorar interfaz para que parezca mas moderna
    /// <summary>
    /// GPU-accelerated xPBD cloth solver using compute shaders.
    /// Runs the full simulation on GPU: predict → constraints → collision → commit.
    ///
    /// When paired with GPUSDFBuilder, the SDF volume is read directly on GPU
    /// without CPU readback — the biggest performance win.
    ///
    /// 3-tier fallback: GPU (this) → Native C++ → Managed C#.
    /// </summary>
    struct Int2
    {
        public int x, y;
    }

    class GPUClothSolver : IDisposable
    {
        // ── Compute shader & kernels ────────────────────────────────
        private ComputeShader _shader;
        private int _kPredict, _kSolveEdges, _kSolveBends, _kSolveCollision, _kCommit;

        // ── Per-vertex buffers (stride 12 = float3) ─────────────────
        private ComputeBuffer _posBuf;
        private ComputeBuffer _velBuf;
        private ComputeBuffer _predBuf;
        private ComputeBuffer _invMassBuf;
        private ComputeBuffer _isPinnedBuf;

        // ── Topology buffers (stride 8 = int2 for pairs) ────────────
        private ComputeBuffer _edgeBuf;
        private ComputeBuffer _edgeRestLenBuf;
        private ComputeBuffer _bendBuf;
        private ComputeBuffer _bendRestLenBuf;

        // ── Collider buffer (stride 60 = NativeCollider) ────────────
        private ComputeBuffer _colliderBuf;

        // ── Dummy buffers (bound when real data is unavailable) ─────
        private ComputeBuffer _dummyColliderBuf;
        private ComputeBuffer _dummySdfBuf;

        // ── Topology metadata (CPU-side) ────────────────────────────
        private int[] _edgeGroupOffsets;
        private int[] _edgeGroupCounts;
        private int   _edgeGroupCount;
        private int[] _bendGroupOffsets;
        private int[] _bendGroupCounts;
        private int   _bendGroupCount;

        // ── Capacities (grow-only) ──────────────────────────────────
        private int _allocVerts;
        private int _allocEdges;
        private int _allocBends;
        private int _allocColliders;

        // ── Scratch arrays ──────────────────────────────────────────
        private int[] _isPinnedTemp;

        // ── State ───────────────────────────────────────────────────
        private bool _disposed;
        public bool IsReady { get; private set; }

        // ── Stride constants ────────────────────────────────────────
        private const int Float3Stride    = 12;
        private const int Int2Stride      = 8;
        private const int ColliderStride  = 60; // NativeCollider: 15 fields × 4 bytes

        // ── AssetBundle (shared with GPUSDFBuilder) ─────────────────
        private const string ShaderAssetName = "ClothSimCompute";

        // ================================================================
        // Factory
        // ================================================================

        /// <summary>
        /// Try to create a GPU cloth solver. Returns null if compute shaders
        /// are unavailable or the shader asset bundle is missing.
        /// </summary>
        public static GPUClothSolver TryCreate()
        {
            if (!SystemInfo.supportsComputeShaders)
                return null;

            ComputeShader cs = LoadComputeShader();
            if (cs == null)
                return null;

            var solver = new GPUClothSolver();
            solver._shader = cs;

            try
            {
                solver._kPredict        = cs.FindKernel("ClothPredict");
                solver._kSolveEdges     = cs.FindKernel("ClothSolveEdges");
                solver._kSolveBends     = cs.FindKernel("ClothSolveBends");
                solver._kSolveCollision = cs.FindKernel("ClothSolveCollision");
                solver._kCommit         = cs.FindKernel("ClothCommit");
            }
            catch (Exception)
            {
                return null;
            }

            // Dummy buffers (1-element) to bind when real data is absent
            solver._dummyColliderBuf = new ComputeBuffer(1, ColliderStride);
            solver._dummySdfBuf      = new ComputeBuffer(1, sizeof(float));
            solver.IsReady = true;
            return solver;
        }

        private static ComputeShader LoadComputeShader()
        {
            return ComputeBundleLoader.LoadShader(ShaderAssetName);
        }

        // ================================================================
        // Topology upload (call when mesh activates or pins change)
        // ================================================================

        /// <summary>
        /// Upload edge/bend topology to GPU. Pre-groups edges by color for
        /// per-group dispatch. Call once on mesh activation.
        /// </summary>
        public void UploadTopology(ClothMeshState s)
        {
            // ── Edges ──
            if (s.EdgeColorGroups != null && s.EdgeColorGroups.Length > 0)
            {
                int totalEdges = 0;
                foreach (int[] g in s.EdgeColorGroups) totalEdges += g.Length;

                Int2[] structPairs = new Int2[totalEdges];
                float[] flatRest = new float[totalEdges];
                _edgeGroupOffsets = new int[s.EdgeColorGroups.Length];
                _edgeGroupCounts  = new int[s.EdgeColorGroups.Length];
                _edgeGroupCount   = s.EdgeColorGroups.Length;

                int off = 0;
                for (int c = 0; c < s.EdgeColorGroups.Length; c++)
                {
                    int[] group = s.EdgeColorGroups[c];
                    _edgeGroupOffsets[c] = off;
                    _edgeGroupCounts[c]  = group.Length;
                    for (int gi = 0; gi < group.Length; gi++)
                    {
                        int eIdx = group[gi];
                        structPairs[off + gi] = new Int2 {
                            x = s.Edges[eIdx * 2],
                            y = s.Edges[eIdx * 2 + 1]
                        };
                        flatRest[off + gi] = s.RestEdgeLen[eIdx];
                    }
                    off += group.Length;
                }

                EnsureEdgeBuffers(totalEdges);
                _edgeBuf.SetData(structPairs);
                _edgeRestLenBuf.SetData(flatRest);
            }
            else
            {
                // No color groups: single group with all edges
                int totalEdges = s.Edges.Length / 2;
                _edgeGroupOffsets = new int[] { 0 };
                _edgeGroupCounts  = new int[] { totalEdges };
                _edgeGroupCount   = 1;

                Int2[] structPairs = new Int2[totalEdges];
                for (int i = 0; i < totalEdges; i++)
                {
                    structPairs[i] = new Int2 {
                        x = s.Edges[i * 2],
                        y = s.Edges[i * 2 + 1]
                    };
                }

                EnsureEdgeBuffers(totalEdges);
                _edgeBuf.SetData(structPairs);
                _edgeRestLenBuf.SetData(s.RestEdgeLen);
            }

            // ── Bends ──
            if (s.BendPairs != null && s.BendPairs.Length > 0)
            {
                if (s.BendColorGroups != null && s.BendColorGroups.Length > 0)
                {
                    int totalBends = 0;
                    foreach (int[] g in s.BendColorGroups) totalBends += g.Length;

                    Int2[] structPairs = new Int2[totalBends];
                    float[] flatRest = new float[totalBends];
                    _bendGroupOffsets = new int[s.BendColorGroups.Length];
                    _bendGroupCounts  = new int[s.BendColorGroups.Length];
                    _bendGroupCount   = s.BendColorGroups.Length;

                    int off = 0;
                    for (int c = 0; c < s.BendColorGroups.Length; c++)
                    {
                        int[] group = s.BendColorGroups[c];
                        _bendGroupOffsets[c] = off;
                        _bendGroupCounts[c]  = group.Length;
                        for (int gi = 0; gi < group.Length; gi++)
                        {
                            int bIdx = group[gi];
                            structPairs[off + gi] = new Int2 {
                                x = s.BendPairs[bIdx * 2],
                                y = s.BendPairs[bIdx * 2 + 1]
                            };
                            flatRest[off + gi] = s.RestBendLen[bIdx];
                        }
                        off += group.Length;
                    }

                    EnsureBendBuffers(totalBends);
                    _bendBuf.SetData(structPairs);
                    _bendRestLenBuf.SetData(flatRest);
                }
                else
                {
                    int totalBends = s.BendPairs.Length / 2;
                    _bendGroupOffsets = new int[] { 0 };
                    _bendGroupCounts  = new int[] { totalBends };
                    _bendGroupCount   = 1;

                    Int2[] structPairs = new Int2[totalBends];
                    for (int i = 0; i < totalBends; i++)
                    {
                        structPairs[i] = new Int2 {
                            x = s.BendPairs[i * 2],
                            y = s.BendPairs[i * 2 + 1]
                        };
                    }

                    EnsureBendBuffers(totalBends);
                    _bendBuf.SetData(structPairs);
                    _bendRestLenBuf.SetData(s.RestBendLen);
                }
            }
            else
            {
                _bendGroupCount = 0;
            }
        }

        // ================================================================
        // Simulate (runs all substeps on GPU, single upload/readback)
        // ================================================================

        /// <summary>
        /// Run the full xPBD simulation on GPU for all substeps.
        /// Positions and velocities are uploaded once, all substep dispatches
        /// chain on GPU, then results are read back once.
        /// </summary>
        public void Simulate(
            Vector3[] positions, Vector3[] velocities,
            float[] invMass, bool[] isPinned,
            int vertCount, int padFreeCount,
            NativeBridge.NativeCollider[] colliders, int colCount,
            float gravity, float dt, int substeps, int iterations,
            float tildedCompliance, float bendCompliance,
            float pressScale, float thickness,
            float damp, float maxSpeed, float frictionZone,
            bool useSDF, ComputeBuffer sdfBuffer,
            float sdfOriginX, float sdfOriginY, float sdfOriginZ,
            int sdfResX, int sdfResY, int sdfResZ,
            float sdfCellSize, float sdfMaxDist)
        {
            // ── 1. Upload per-vertex state ──
            EnsureVertBuffers(vertCount);

            _posBuf.SetData(positions, 0, 0, vertCount);
            _velBuf.SetData(velocities, 0, 0, vertCount);
            _invMassBuf.SetData(invMass, 0, 0, vertCount);

            if (_isPinnedTemp == null || _isPinnedTemp.Length < vertCount)
                _isPinnedTemp = new int[vertCount];
            for (int i = 0; i < vertCount; i++)
                _isPinnedTemp[i] = isPinned[i] ? 1 : 0;
            _isPinnedBuf.SetData(_isPinnedTemp, 0, 0, vertCount);

            // ── 2. Upload colliders ──
            ComputeBuffer colBuf = _dummyColliderBuf;
            if (colCount > 0)
            {
                EnsureColliderBuffer(colCount);
                _colliderBuf.SetData(colliders, 0, 0, colCount);
                colBuf = _colliderBuf;
            }

            // ── 3. SDF buffer ──
            ComputeBuffer sdfBuf = (useSDF && sdfBuffer != null) ? sdfBuffer : _dummySdfBuf;

            // ── 4. Bind all buffers ──
            BindAllBuffers(colBuf, sdfBuf);

            // ── 5. Set shared uniforms ──
            _shader.SetInt("_VertCount", vertCount);
            _shader.SetInt("_ColCount", colCount);
            _shader.SetFloat("_Thickness", thickness);
            _shader.SetFloat("_TildedCompliance", tildedCompliance);
            _shader.SetFloat("_BendCompliance", bendCompliance);
            _shader.SetFloat("_PressScale", pressScale);
            _shader.SetInt("_UseSDF", useSDF ? 1 : 0);

            if (useSDF)
            {
                _shader.SetFloat("_SDFOriginX", sdfOriginX);
                _shader.SetFloat("_SDFOriginY", sdfOriginY);
                _shader.SetFloat("_SDFOriginZ", sdfOriginZ);
                _shader.SetInt("_SDFResX", sdfResX);
                _shader.SetInt("_SDFResY", sdfResY);
                _shader.SetInt("_SDFResZ", sdfResZ);
                _shader.SetFloat("_SDFCellSize", sdfCellSize);
                _shader.SetFloat("_SDFInvCellSize", sdfCellSize > 0 ? 1f / sdfCellSize : 0f);
                _shader.SetFloat("_SDFMaxDist", sdfMaxDist);
                _shader.SetFloat("_FrictionZone", frictionZone);
            }

            int vertGroups = padFreeCount > 0 ? (padFreeCount / 256) : 1;

            // ── 6. Substep loop (all on GPU — no CPU round-trips) ──
            float subDt = dt / Mathf.Max(1, substeps);
            float invDt = 1f / subDt;

            for (int sub = 0; sub < substeps; sub++)
            {
                // Predict
                _shader.SetFloat("_Gravity", gravity);
                _shader.SetFloat("_Dt", subDt);
                _shader.Dispatch(_kPredict, vertGroups, 1, 1);

                // Constraint iterations
                for (int iter = 0; iter < iterations; iter++)
                {
                    // Edges per color group
                    for (int c = 0; c < _edgeGroupCount; c++)
                    {
                        _shader.SetInt("_GroupOffset", _edgeGroupOffsets[c]);
                        _shader.SetInt("_GroupCount", _edgeGroupCounts[c]);
                        int eg = (_edgeGroupCounts[c] + 255) / 256;
                        _shader.Dispatch(_kSolveEdges, Mathf.Max(1, eg), 1, 1);
                    }

                    // Bends per color group
                    for (int c = 0; c < _bendGroupCount; c++)
                    {
                        _shader.SetInt("_GroupOffset", _bendGroupOffsets[c]);
                        _shader.SetInt("_GroupCount", _bendGroupCounts[c]);
                        int bg = (_bendGroupCounts[c] + 255) / 256;
                        _shader.Dispatch(_kSolveBends, Mathf.Max(1, bg), 1, 1);
                    }
                }

                // Collision
                _shader.Dispatch(_kSolveCollision, vertGroups, 1, 1);

                // Commit
                _shader.SetFloat("_InvDt", invDt);
                _shader.SetFloat("_Damp", damp);
                _shader.SetFloat("_MaxSpeed", maxSpeed);
                _shader.Dispatch(_kCommit, vertGroups, 1, 1);
            }

            // ── 7. Readback results (single sync point) ──
            _posBuf.GetData(positions, 0, 0, vertCount);
            _velBuf.GetData(velocities, 0, 0, vertCount);
        }

        // ================================================================
        // Buffer management
        // ================================================================

        private void BindAllBuffers(ComputeBuffer colBuf, ComputeBuffer sdfBuf)
        {
            // Per-vertex buffers (used by all kernels)
            int[] allK = { _kPredict, _kSolveEdges, _kSolveBends, _kSolveCollision, _kCommit };
            foreach (int k in allK)
            {
                _shader.SetBuffer(k, "_Positions", _posBuf);
                _shader.SetBuffer(k, "_Velocities", _velBuf);
                _shader.SetBuffer(k, "_PredPositions", _predBuf);
                _shader.SetBuffer(k, "_InvMass", _invMassBuf);
                _shader.SetBuffer(k, "_IsPinned", _isPinnedBuf);
            }

            // Topology
            _shader.SetBuffer(_kSolveEdges, "_EdgePairs", _edgeBuf);
            _shader.SetBuffer(_kSolveEdges, "_EdgeRestLen", _edgeRestLenBuf);
            if (_bendGroupCount > 0 && _bendBuf != null)
            {
                _shader.SetBuffer(_kSolveBends, "_BendPairs", _bendBuf);
                _shader.SetBuffer(_kSolveBends, "_BendRestLen", _bendRestLenBuf);
            }

            // Colliders
            _shader.SetBuffer(_kSolveCollision, "_Colliders", colBuf);

            // SDF (collision + commit kernels)
            _shader.SetBuffer(_kSolveCollision, "_SDFData", sdfBuf);
            _shader.SetBuffer(_kCommit, "_SDFData", sdfBuf);
        }

        private void EnsureVertBuffers(int count)
        {
            if (_posBuf != null && _allocVerts >= count) return;

            _posBuf?.Release();
            _velBuf?.Release();
            _predBuf?.Release();
            _invMassBuf?.Release();
            _isPinnedBuf?.Release();

            _posBuf      = new ComputeBuffer(count, Float3Stride);
            _velBuf      = new ComputeBuffer(count, Float3Stride);
            _predBuf     = new ComputeBuffer(count, Float3Stride);
            _invMassBuf  = new ComputeBuffer(count, sizeof(float));
            _isPinnedBuf = new ComputeBuffer(count, sizeof(int));
            _allocVerts  = count;
        }

        private void EnsureEdgeBuffers(int count)
        {
            if (_edgeBuf != null && _allocEdges >= count) return;

            _edgeBuf?.Release();
            _edgeRestLenBuf?.Release();

            _edgeBuf        = new ComputeBuffer(count, Int2Stride);
            _edgeRestLenBuf = new ComputeBuffer(count, sizeof(float));
            _allocEdges     = count;
        }

        private void EnsureBendBuffers(int count)
        {
            if (_bendBuf != null && _allocBends >= count) return;

            _bendBuf?.Release();
            _bendRestLenBuf?.Release();

            _bendBuf        = new ComputeBuffer(count, Int2Stride);
            _bendRestLenBuf = new ComputeBuffer(count, sizeof(float));
            _allocBends     = count;
        }

        private void EnsureColliderBuffer(int count)
        {
            if (_colliderBuf != null && _allocColliders >= count) return;

            _colliderBuf?.Release();
            _colliderBuf    = new ComputeBuffer(count, ColliderStride);
            _allocColliders = count;
        }

        // ================================================================
        // Cleanup
        // ================================================================

        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;

            _posBuf?.Release();
            _velBuf?.Release();
            _predBuf?.Release();
            _invMassBuf?.Release();
            _isPinnedBuf?.Release();
            _edgeBuf?.Release();
            _edgeRestLenBuf?.Release();
            _bendBuf?.Release();
            _bendRestLenBuf?.Release();
            _colliderBuf?.Release();
            _dummyColliderBuf?.Release();
            _dummySdfBuf?.Release();

            _posBuf = null;
            _velBuf = null;
            _predBuf = null;
            _invMassBuf = null;
            _isPinnedBuf = null;
            _edgeBuf = null;
            _edgeRestLenBuf = null;
            _bendBuf = null;
            _bendRestLenBuf = null;
            _colliderBuf = null;
            _dummyColliderBuf = null;
            _dummySdfBuf = null;
        }
    }
}
