using AIChara;
using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace StudioModsMSG
{
    /// <summary>
    /// Redesigned breast soft-body â€“ two virtual breast-center springs (one per side)
    /// replace the old N-per-vertex springs.  Vertices are displaced via projection of
    /// the lag vector onto their outward / tangent directions, amplified quadratically
    /// by radial distance (tip moves far more than base = pole-bending jiggle).
    /// A multi-harmonic traveling surface wave driven by the spring velocity propagates
    /// from base to tip with per-vertex deterministic phase jitter, producing complex
    /// organic skin texture rather than uniform ring waves.
    /// </summary>
    class BreastSoftBodyRuntime : MonoBehaviour
    {
        // -------------------------------------------------------------------
        // Bone helper
        // -------------------------------------------------------------------
        private readonly struct BreastBoneRef
        {
            public readonly int       Index;
            public readonly Transform Transform;
            public BreastBoneRef(int index, Transform transform) { Index = index; Transform = transform; }
        }

        // -------------------------------------------------------------------
        // Per-mesh profile â€“ precomputed per-vertex data only; NO per-vertex state
        // -------------------------------------------------------------------
        private sealed class MeshProfile
        {
            public SkinnedMeshRenderer Renderer;
            public Mesh     RuntimeMesh;
            public Vector3[] BaseVertices;
            public Vector3[] WorkingVertices;
            public int[]     VertexIndices;
            public float[]   VertexMask;      // smooth weight fade (0..1)
            public Vector3[] VertexOutward;   // direction from breast center â†’ vertex (rest pose)
            public Vector3[] VertexTangent;   // lateral axis perpendicular to outward
            public float[]   VertexDist;      // radial distance from breast center, normalised 0..1
            public float[]   VertexSide;      // 0 = left breast, 1 = right breast
            public float[]   VertexJitter;    // deterministic per-vertex phase noise (0..1)
            public bool Valid;
        }

        // -------------------------------------------------------------------
        // Runtime fields
        // -------------------------------------------------------------------
        private ChaControl               chaCtrl;
        private readonly List<MeshProfile>  profiles         = new List<MeshProfile>();
        private readonly List<Transform>    leftBreastBones  = new List<Transform>();
        private readonly List<Transform>    rightBreastBones = new List<Transform>();

        // Two virtual breast-center spring states (world-space displacement from bone center)
        private Vector3 leftLag      = Vector3.zero;
        private Vector3 leftLagVel   = Vector3.zero;
        private Vector3 rightLag     = Vector3.zero;
        private Vector3 rightLagVel  = Vector3.zero;

        // Ripple phase accumulators â€“ advance proportional to spring speed
        private float leftRipplePhase  = 0f;
        private float rightRipplePhase = 0f;

        // Previous bone centers for velocity estimation
        private Vector3 prevLeftCenter;
        private Vector3 prevRightCenter;
        private bool    centersInitialized;
        private int     validateCooldown;

        // -------------------------------------------------------------------
        // Public interface
        // -------------------------------------------------------------------
        public void Attach(ChaControl control)
        {
            chaCtrl = control;
            RebuildProfiles();
        }

        private void OnEnable()
        {
            if (chaCtrl != null) RebuildProfiles();
        }

        private void OnDisable()
        {
            ResetState();
        }

        // -------------------------------------------------------------------
        // Per-frame update
        // -------------------------------------------------------------------
        private void LateUpdate()
        {
            if (!CanRun()) return;

            if (profiles.Count == 0)
            {
                RebuildProfiles();
                if (profiles.Count == 0) return;
            }

            validateCooldown--;
            if (validateCooldown <= 0)
            {
                validateCooldown = 60;
                if (NeedsRebuild())
                {
                    RebuildProfiles();
                    if (profiles.Count == 0) return;
                }
            }

            float dt = Mathf.Clamp(Time.deltaTime, 0.0001f, 0.05f);

            float intensity  = Mathf.Clamp01(StudioCharaEditor.BreastSoftBodyIntensity.Value);
            float stiffness  = Mathf.Max(0.1f,  StudioCharaEditor.BreastSoftBodyStiffness.Value);
            float damping    = Mathf.Max(0.1f,  StudioCharaEditor.BreastSoftBodyDamping.Value);
            float influence  = Mathf.Max(0f,    StudioCharaEditor.BreastSoftBodyMotionInfluence.Value);
            float maxOffset  = Mathf.Clamp(StudioCharaEditor.BreastSoftBodyMaxOffset.Value, 0.001f, 0.15f);
            float lateral    = Mathf.Max(0f,    StudioCharaEditor.BreastSoftBodyLateralStrength.Value);
            float waveSpeed  = Mathf.Max(0f,    StudioCharaEditor.BreastSoftBodyWaveSpeed.Value);
            float secAmp     = Mathf.Clamp01(StudioCharaEditor.BreastSoftBodySecondaryAmplitude.Value);
            float gravitySag = Mathf.Max(0f,    StudioCharaEditor.BreastSoftBodyGravitySag.Value);

            // ---- Current bone centers ----------------------------------------
            Vector3 leftCenter  = GetBoneCenter(leftBreastBones);
            Vector3 rightCenter = GetBoneCenter(rightBreastBones);

            if (!centersInitialized)
            {
                prevLeftCenter  = leftCenter;
                prevRightCenter = rightCenter;
                centersInitialized = true;
            }

            // ---- Bone velocity (world space) ----------------------------------
            Vector3 leftBoneVel  = (leftCenter  - prevLeftCenter)  / dt;
            Vector3 rightBoneVel = (rightCenter - prevRightCenter) / dt;

            // ---- Integrate virtual breast springs ----------------------------
            // The "lag" vector represents how far behind the virtual breast mass
            // has fallen relative to the real bone center.  When the chest moves,
            // the spring creates an inertial force causing jiggle.
            IntegrateBreastSpring(ref leftLag,  ref leftLagVel,  leftBoneVel,
                                  stiffness, damping, influence, maxOffset, dt);
            IntegrateBreastSpring(ref rightLag, ref rightLagVel, rightBoneVel,
                                  stiffness, damping, influence, maxOffset, dt);

            // ---- Advance ripple phases from spring speed ---------------------
            // The faster the virtual centers oscillate, the faster the wave
            // propagates across the breast surface.
            leftRipplePhase  += leftLagVel.magnitude  * waveSpeed * 9f * dt;
            rightRipplePhase += rightLagVel.magnitude * waveSpeed * 9f * dt;
            const float phaseMax = 6283.2f;   // ~1000 Ã— 2Ï€  (prevent float drift)
            if (leftRipplePhase  > phaseMax) leftRipplePhase  -= phaseMax;
            if (rightRipplePhase > phaseMax) rightRipplePhase -= phaseMax;

            // ---- Apply to all mesh profiles ----------------------------------
            for (int p = 0; p < profiles.Count; p++)
            {
                MeshProfile profile = profiles[p];
                if (!profile.Valid || profile.Renderer == null || profile.RuntimeMesh == null)
                    continue;

                Transform tf = profile.Renderer.transform;

                // World-space vectors â†’ renderer local space
                Vector3 localLeftLag  = tf.InverseTransformVector(leftLag);
                Vector3 localRightLag = tf.InverseTransformVector(rightLag);
                Vector3 localDown     = tf.InverseTransformDirection(Vector3.down);

                for (int i = 0; i < profile.VertexIndices.Length; i++)
                {
                    int   vi      = profile.VertexIndices[i];
                    float mk      = profile.VertexMask[i] * intensity;
                    if (mk <= 0.0001f)
                    {
                        profile.WorkingVertices[vi] = profile.BaseVertices[vi];
                        continue;
                    }

                    Vector3 outward = profile.VertexOutward[i];
                    Vector3 tangent = profile.VertexTangent[i];
                    float   dist    = profile.VertexDist[i];    // 0 = base  â€¦ 1 = tip
                    float   side    = profile.VertexSide[i];    // 0 = left  â€¦ 1 = right
                    float   jitter  = profile.VertexJitter[i];  // 0..1 deterministic noise

                    // Blend lag between left / right breast
                    Vector3 localLag = Vector3.Lerp(localLeftLag, localRightLag, side);

                    // ----------------------------------------------------------
                    // PRIMARY DISPLACEMENT â€“ pole-bend model
                    //
                    // Project the lag vector onto the vertex's outward normal.
                    // The quadratic distance factor (distSq) gives the tip ~4Ã—
                    // more movement than the midpoint â€” a natural pendulum feel
                    // where the breast hangs / sways from its attachment base.
                    // ----------------------------------------------------------
                    float distSq  = dist * dist;
                    float primOut = Vector3.Dot(localLag, outward) * distSq;
                    float primLat = Vector3.Dot(localLag, tangent) * dist * lateral;

                    // ----------------------------------------------------------
                    // SURFACE RIPPLE â€“ multi-harmonic traveling wave
                    //
                    // Phase ramps with time (blendPhase) and decreases with dist
                    // so outer vertices lag inner ones â†’ wave travels outward.
                    // Amplitude âˆ distSq so the wave emerges at the base and
                    // grows toward the tip (like a stone dropped at the center).
                    //
                    // Three harmonics at different spatial frequencies and time
                    // ratios create a complex interference pattern.  Per-vertex
                    // jitter (converted to radians) breaks perfect ring symmetry
                    // into organic surface micro-texture.
                    //
                    // Gate: rippleEnv âˆ lagMag so the surface is quiet when idle.
                    // ----------------------------------------------------------
                    float blendPhase  = Mathf.Lerp(leftRipplePhase, rightRipplePhase, side);
                    float jRad        = jitter * 3.14159f;   // 0..Ï€ per-vertex phase noise
                    const float kF1   = 4.5f;   // fundamental spatial frequency
                    const float kF2   = 8.3f;   // second harmonic (â‰ˆ 1.84 Ã— F1)
                    const float kF3   = 2.1f;   // sub-harmonic slow roll

                    float wave1 = Mathf.Sin(blendPhase        - dist * kF1 + jRad);
                    float wave2 = Mathf.Sin(blendPhase * 1.55f - dist * kF2 + jRad * 1.7f);
                    float wave3 = Mathf.Sin(blendPhase * 0.38f - dist * kF3 + jRad * 0.5f);

                    float lagMag   = localLag.magnitude;
                    float envelope = distSq * lagMag;

                    float ripple = (wave1 * 0.55f + wave2 * 0.28f + wave3 * 0.17f)
                                 * envelope * maxOffset * waveSpeed * 0.10f;

                    // ----------------------------------------------------------
                    // SECONDARY WOBBLE â€“ fast parasitic oscillation
                    //
                    // A phase-offset wave at a different time-ratio creates a
                    // high-frequency "wobble on top of the jiggle" layered over
                    // the primary bounce.
                    // ----------------------------------------------------------
                    float secWave  = Mathf.Sin(blendPhase * 2.4f - dist * kF1 * 1.8f + jRad * 2.1f);
                    float secondary = secWave * dist * lagMag * secAmp * maxOffset * 0.20f;

                    // ----------------------------------------------------------
                    // GRAVITY SAG â€“ static downward droop
                    //
                    // Only applied to normals that have a downward component, so
                    // the underside of the breast droops more than the top.
                    // ----------------------------------------------------------
                    float downDot = Mathf.Max(0f, Vector3.Dot(localDown, outward));
                    float sag     = downDot * gravitySag * mk;

                    // ----------------------------------------------------------
                    // COMPOSE
                    // ----------------------------------------------------------
                    float totalOut = (primOut + ripple + secondary) * mk + sag;
                    float totalLat = primLat * mk;

                    float cap = maxOffset * 3f;
                    totalOut = Mathf.Clamp(totalOut, -cap, cap);
                    totalLat = Mathf.Clamp(totalLat, -cap * 0.55f, cap * 0.55f);

                    profile.WorkingVertices[vi] = profile.BaseVertices[vi]
                        + outward * totalOut
                        + tangent * totalLat;
                }

                profile.RuntimeMesh.vertices = profile.WorkingVertices;
                profile.RuntimeMesh.UploadMeshData(false);
            }

            prevLeftCenter  = leftCenter;
            prevRightCenter = rightCenter;
        }

        // -------------------------------------------------------------------
        // Virtual breast spring integrator
        //
        // lag    = how far behind the virtual mass has fallen (world space)
        // lagVel = velocity of the virtual mass
        //
        // External force is  -boneVel * influence  (pseudo-inertial: the mass
        // "lags" in the opposite direction when the chest accelerates).
        // -------------------------------------------------------------------
        private static void IntegrateBreastSpring(
            ref Vector3 lag, ref Vector3 lagVel,
            Vector3 boneVel,
            float stiffness, float damping, float influence,
            float maxOffset, float dt)
        {
            Vector3 force = -boneVel * influence   // inertial pseudo-force
                          - lag      * stiffness   // spring restoring force
                          - lagVel   * damping;    // velocity damping

            lagVel += force * dt;
            lag    += lagVel * dt;

            float len = lag.magnitude;
            if (len > maxOffset)
            {
                lag    = (lag / len) * maxOffset;
                lagVel *= 0.3f;   // bleed off excess energy at the limit
            }
        }

        // -------------------------------------------------------------------
        // Lifecycle helpers
        // -------------------------------------------------------------------
        private bool CanRun()
        {
            if (chaCtrl == null || chaCtrl.gameObject == null) return false;
            if (!StudioCharaEditor.BreastSoftBodyEnabled.Value)  return false;
            if (chaCtrl.sex != 1) return false;   // HS2 female sex == 1
            return true;
        }

        private bool NeedsRebuild()
        {
            if (profiles.Count == 0) return true;
            for (int i = 0; i < profiles.Count; i++)
            {
                MeshProfile pr = profiles[i];
                if (pr.Renderer == null || pr.RuntimeMesh == null) return true;
                if (pr.Renderer.sharedMesh != pr.RuntimeMesh)      return true;
            }
            return leftBreastBones.Count == 0 || rightBreastBones.Count == 0;
        }

        private void ResetState()
        {
            profiles.Clear();
            leftBreastBones.Clear();
            rightBreastBones.Clear();
            leftLag    = rightLag    = Vector3.zero;
            leftLagVel = rightLagVel = Vector3.zero;
            leftRipplePhase = rightRipplePhase = 0f;
            centersInitialized = false;
        }

        // -------------------------------------------------------------------
        // Profile build
        // -------------------------------------------------------------------
        private void RebuildProfiles()
        {
            ResetState();
            validateCooldown = 60;
            if (!CanRun()) return;

            SkinnedMeshRenderer[] renderers =
                chaCtrl.GetComponentsInChildren<SkinnedMeshRenderer>(true);
            if (renderers == null || renderers.Length == 0) return;

            float weightThreshold =
                Mathf.Clamp(StudioCharaEditor.BreastSoftBodyWeightThreshold.Value, 0.005f, 0.5f);

            for (int r = 0; r < renderers.Length; r++)
            {
                SkinnedMeshRenderer smr = renderers[r];
                if (smr == null || smr.sharedMesh == null) continue;

                Mesh        source   = smr.sharedMesh;
                BoneWeight[] weights = source.boneWeights;
                Transform[]  bones   = smr.bones;
                Vector3[]    srcVerts = source.vertices;
                if (weights == null || weights.Length == 0 ||
                    bones   == null || bones.Length   == 0 ||
                    srcVerts == null || srcVerts.Length == 0)
                    continue;

                // ---- Gather breast bone references ---------------------------
                var leftRefs  = new List<BreastBoneRef>();
                var rightRefs = new List<BreastBoneRef>();

                for (int bi = 0; bi < bones.Length; bi++)
                {
                    Transform bone = bones[bi];
                    if (bone == null) continue;
                    string bn = bone.name;
                    if (string.IsNullOrEmpty(bn) || !bn.Contains("cf_J_Mune")) continue;

                    if (bn.EndsWith("_L", StringComparison.Ordinal))
                    {
                        leftRefs.Add(new BreastBoneRef(bi, bone));
                        if (!leftBreastBones.Contains(bone)) leftBreastBones.Add(bone);
                    }
                    else if (bn.EndsWith("_R", StringComparison.Ordinal))
                    {
                        rightRefs.Add(new BreastBoneRef(bi, bone));
                        if (!rightBreastBones.Contains(bone)) rightBreastBones.Add(bone);
                    }
                }

                if (leftRefs.Count == 0 || rightRefs.Count == 0) continue;

                // ---- Bone-index lookup maps ----------------------------------
                var leftMap  = new Dictionary<int, float>();
                var rightMap = new Dictionary<int, float>();
                for (int k = 0; k < leftRefs.Count;  k++) leftMap[leftRefs[k].Index]  = 1f;
                for (int k = 0; k < rightRefs.Count; k++) rightMap[rightRefs[k].Index] = 1f;

                // Rest-pose breast centers in renderer local space
                Vector3 leftCenterLocal  = AverageLocalPoint(smr.transform,
                    leftRefs.Select(x => x.Transform));
                Vector3 rightCenterLocal = AverageLocalPoint(smr.transform,
                    rightRefs.Select(x => x.Transform));

                // ---- Collect influenced vertices -----------------------------
                var vIdx  = new List<int>();
                var vMask = new List<float>();
                var vOut  = new List<Vector3>();
                var vTan  = new List<Vector3>();
                var vDist = new List<float>();
                var vSide = new List<float>();

                int usable = Mathf.Min(weights.Length, srcVerts.Length);
                for (int vi = 0; vi < usable; vi++)
                {
                    BoneWeight bw = weights[vi];
                    float lw = 0f, rw = 0f;
                    AccumWeight(bw.boneIndex0, bw.weight0, leftMap, rightMap, ref lw, ref rw);
                    AccumWeight(bw.boneIndex1, bw.weight1, leftMap, rightMap, ref lw, ref rw);
                    AccumWeight(bw.boneIndex2, bw.weight2, leftMap, rightMap, ref lw, ref rw);
                    AccumWeight(bw.boneIndex3, bw.weight3, leftMap, rightMap, ref lw, ref rw);

                    float total = lw + rw;
                    if (total < weightThreshold) continue;

                    bool    isLeft  = lw >= rw;
                    Vector3 center  = isLeft ? leftCenterLocal : rightCenterLocal;
                    Vector3 delta   = srcVerts[vi] - center;
                    float   rawDist = delta.magnitude;
                    Vector3 outward = rawDist > 0.000001f ? delta / rawDist : Vector3.up;

                    // Tangent: perpendicular to outward in a stable way
                    Vector3 refVec  = Mathf.Abs(outward.y) < 0.9f ? Vector3.up : Vector3.forward;
                    Vector3 tangent = Vector3.Cross(outward, refVec).normalized;

                    vIdx.Add(vi);
                    vMask.Add(Mathf.Clamp01(
                        (total - weightThreshold) / Mathf.Max(0.001f, 1f - weightThreshold)));
                    vOut.Add(outward);
                    vTan.Add(tangent);
                    vDist.Add(rawDist);
                    vSide.Add(isLeft ? 0f : 1f);
                }

                if (vIdx.Count == 0) continue;

                // ---- Normalise distances to 0..1 ----------------------------
                float maxDist = 0f;
                for (int k = 0; k < vDist.Count; k++)
                    maxDist = Mathf.Max(maxDist, vDist[k]);
                if (maxDist < 0.0001f) maxDist = 1f;

                int count = vIdx.Count;
                float[] distNorm = new float[count];
                for (int k = 0; k < count; k++)
                    distNorm[k] = vDist[k] / maxDist;

                // ---- Deterministic per-vertex phase jitter -------------------
                // A simple hash isolates each vertex's phase without any state.
                // Using the golden-angle-like constant 127.1 distributes values
                // evenly so the jitter doesn't cluster.
                float[] jitter = new float[count];
                for (int k = 0; k < count; k++)
                {
                    float h = Mathf.Sin((float)vIdx[k] * 127.1f) * 43758.5453f;
                    jitter[k] = h - Mathf.Floor(h);   // fract(h) â†’ 0..1
                }

                // ---- Instantiate writable runtime mesh ----------------------
                Mesh runtime   = UnityEngine.Object.Instantiate(source);
                runtime.name   = source.name + "_BreastSoftBody";
                smr.sharedMesh = runtime;

                profiles.Add(new MeshProfile
                {
                    Renderer        = smr,
                    RuntimeMesh     = runtime,
                    BaseVertices    = runtime.vertices,
                    WorkingVertices = runtime.vertices,
                    VertexIndices   = vIdx.ToArray(),
                    VertexMask      = vMask.ToArray(),
                    VertexOutward   = vOut.ToArray(),
                    VertexTangent   = vTan.ToArray(),
                    VertexDist      = distNorm,
                    VertexSide      = vSide.ToArray(),
                    VertexJitter    = jitter,
                    Valid           = true,
                });
            }

            if (StudioCharaEditor.VerboseMessage.Value)
            {
                int total = profiles.Sum(x => x.VertexIndices != null ? x.VertexIndices.Length : 0);
                StudioCharaEditor.Logger.LogInfo(
                    "BreastSoftBody rebuild: " + profiles.Count + " profiles, " + total + " vertices");
            }
        }

        // -------------------------------------------------------------------
        // Static mini-helpers
        // -------------------------------------------------------------------
        private static void AccumWeight(
            int boneIndex, float value,
            Dictionary<int, float> lm, Dictionary<int, float> rm,
            ref float lw, ref float rw)
        {
            if (value <= 0f) return;
            if (lm.ContainsKey(boneIndex)) lw += value;
            if (rm.ContainsKey(boneIndex)) rw += value;
        }

        private static Vector3 AverageLocalPoint(Transform localOf, IEnumerable<Transform> points)
        {
            Vector3 sum   = Vector3.zero;
            int     count = 0;
            foreach (Transform t in points)
            {
                if (t == null) continue;
                sum += localOf.InverseTransformPoint(t.position);
                count++;
            }
            return count > 0 ? sum / count : Vector3.zero;
        }

        private static Vector3 GetBoneCenter(List<Transform> bones)
        {
            if (bones == null || bones.Count == 0) return Vector3.zero;
            Vector3 sum   = Vector3.zero;
            int     count = 0;
            for (int i = 0; i < bones.Count; i++)
            {
                if (bones[i] == null) continue;
                sum += bones[i].position;
                count++;
            }
            return count > 0 ? sum / count : Vector3.zero;
        }
    }
}
