using AIChara;
using System;
using System.Collections.Generic;
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
    class ClothSoftBodyRuntime : MonoBehaviour
    {
        private struct MirrorCollider
        {
            public Vector3 Center;
            public float Radius;
            public float Height; 
            public Vector3 Direction;
            public bool IsCapsule;
        }

        private List<MirrorCollider> mirrorColliders = new List<MirrorCollider>();

        private void UpdateMirrorColliders()
        {
            mirrorColliders.Clear();
            if (chaCtrl == null) return;

            // Buscamos los componentes de Dynamic Bone en el personaje
            var dbColliders = chaCtrl.GetComponentsInChildren<DynamicBoneColliderBase>(true);

            foreach (var dbCol in dbColliders)
            {
                if (!dbCol.enabled) continue;

                // Intentamos obtener el radio y altura dinámicamente mediante reflexión 
                // o asumiendo campos estándar si no puedes acceder a las clases derivadas
                // Aquí usamos la lógica estándar de estos componentes:
                float radius = 0;
                float height = 0;

                // Acceso seguro a propiedades comunes en implementaciones de DynamicBone
                var type = dbCol.GetType();
                var fRadius = type.GetField("m_Radius");
                var fHeight = type.GetField("m_Height");

                if (fRadius != null) radius = (float)fRadius.GetValue(dbCol);
                if (fHeight != null) height = (float)fHeight.GetValue(dbCol);

                MirrorCollider mc = new MirrorCollider();
                mc.Center = dbCol.transform.TransformPoint(dbCol.m_Center);
                mc.Radius = radius * Mathf.Abs(dbCol.transform.lossyScale.x);
                mc.IsCapsule = height > 0;
                
                if (mc.IsCapsule)
                {
                    mc.Height = height * Mathf.Abs(dbCol.transform.lossyScale.y);
                    Vector3 dir = Vector3.up;
                    if (dbCol.m_Direction == DynamicBoneColliderBase.Direction.X) dir = Vector3.right;
                    else if (dbCol.m_Direction == DynamicBoneColliderBase.Direction.Z) dir = Vector3.forward;
                    mc.Direction = dbCol.transform.TransformDirection(dir);
                }
                
                mirrorColliders.Add(mc);
            }
        }
        // ------------------------------------------------------------------ //
        // Character root tracking (follow when cloth has no bone-weight pins)
        // ------------------------------------------------------------------ //
        private Vector3 chaLastPos      = Vector3.zero;
        private bool    chaLastPosValid = false;

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
        }

        public void DeactivateMesh(ClothMeshState state)
        {
            if (state == null || !state.IsActive) return;
            TeardownMeshState(state);
            state.IsActive = false;
            activeStates.Remove(state);
        }

        // ------------------------------------------------------------------ //
        // Lifecycle
        // ------------------------------------------------------------------ //
        private void OnDisable()
        {
            // Deactivate all on component disable
            for (int i = activeStates.Count - 1; i >= 0; i--)
                DeactivateMesh(activeStates[i]);
        }

        private void LateUpdate()
        {
            if (chaCtrl == null || activeStates.Count == 0) return;

            // Character root movement this frame — used as follow-delta when no pins.
            Vector3 chaPos = chaCtrl.transform.position;
            Vector3 chaMoveDelta = chaLastPosValid ? (chaPos - chaLastPos) : Vector3.zero;
            chaLastPos      = chaPos;
            chaLastPosValid = true;

            float dt = Mathf.Clamp(Time.deltaTime, 0.001f, 0.05f);

            foreach (ClothMeshState state in activeStates)
            {
                if (state == null || state.Renderer == null) continue;
                int   substeps = Mathf.Max(1, state.Params.Substeps);
                float subDt    = dt / substeps;
                Vector3 subCharDelta = chaMoveDelta / substeps;
                for (int sub = 0; sub < substeps; sub++)
                    SimulateStep(state, subDt, subCharDelta);

                if (state.Params.ClothToCloth)
                    ApplyClothToClothCollision(state);

                WriteMesh(state);
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
        private void SimulateStep(ClothMeshState s, float dt, Vector3 chaMoveDelta)
        {
            int n = s.VertCount;
            UpdateMirrorColliders(); // Sincroniza el espejo en cada frame

            Array.Copy(s.Position, s.PrevPosition, n);

            // 1. Skeleton follow
            if (chaMoveDelta.sqrMagnitude > 1e-12f)
                for (int i = 0; i < n; i++)
                    if (!s.IsPinned[i]) { s.Position[i] += chaMoveDelta; s.PrevPosition[i] += chaMoveDelta; }

            // 2. Predict
            float g = s.Params.Gravity;
            for (int i = 0; i < n; i++)
            {
                if (s.IsPinned[i]) { s.PredPosition[i] = s.Position[i]; continue; }
                s.Velocity[i].y += g * dt;
                s.PredPosition[i] = s.Position[i] + s.Velocity[i] * dt;
            }

            // 3. Constraints Iterations
            float tildedCompliance = 1f / Mathf.Max(1e-6f, s.Params.StretchStiffness * dt * dt);
            float thick = s.Params.Thickness;

            for (int iter = 0; iter < s.Params.Iterations; iter++)
            {
                SolveEdges_XPBD(s, tildedCompliance);
                ApplyHardPins(s, s.PredPosition); // Vital para que no se caiga la ropa

                // Colisiones contra el espejo de DynamicBoneColliders
                for (int i = 0; i < n; i++)
                {
                    if (s.IsPinned[i]) continue;
                    Vector3 p = s.PredPosition[i];

                    foreach (var col in mirrorColliders)
                    {
                        Vector3 closest;
                        if (!col.IsCapsule) closest = col.Center;
                        else
                        {
                            float halfH = Mathf.Max(0, col.Height * 0.5f - col.Radius);
                            Vector3 p1 = col.Center + col.Direction * halfH;
                            Vector3 p2 = col.Center - col.Direction * halfH;
                            Vector3 v = p2 - p1;
                            Vector3 w = p - p1;
                            float t = Mathf.Clamp01(Vector3.Dot(w, v) / Vector3.Dot(v, v));
                            closest = p1 + t * v;
                        }

                        Vector3 delta = p - closest;
                        float distSq = delta.sqrMagnitude;
                        float minSep = col.Radius + thick;

                        if (distSq < minSep * minSep)
                        {
                            float dist = Mathf.Sqrt(distSq);
                            Vector3 normal = (dist > 1e-6f) ? (delta / dist) : Vector3.up;
                            p = closest + normal * minSep;
                        }
                    }
                    s.PredPosition[i] = p;
                }
            }

            // Shape retention / compression — pulls free verts toward rest body pose.
            // alpha is framerate-normalised so behaviour is stable across substep counts.
            if (s.Params.Compression > 0f && s.RestBodyLocalPos != null && chaCtrl != null)
            {
                float alpha = Mathf.Clamp01(s.Params.Compression * dt * 60f);
                for (int i = 0; i < n; i++)
                {
                    if (s.IsPinned[i]) continue;
                    Vector3 restWorld = chaCtrl.transform.TransformPoint(s.RestBodyLocalPos[i]);
                    s.PredPosition[i] = Vector3.Lerp(s.PredPosition[i], restWorld, alpha);
                }
            }

            // 4. Commit
            float inv_dt = 1f / dt;
            float damp = Mathf.Clamp01(1f - s.Params.Damping * dt);
            for (int i = 0; i < n; i++)
            {
                if (s.IsPinned[i]) { s.Velocity[i] = Vector3.zero; s.Position[i] = s.PredPosition[i]; continue; }
                s.Velocity[i] = (s.PredPosition[i] - s.Position[i]) * inv_dt * damp;
                s.Position[i] = s.PredPosition[i];
            }
        }
        private static void SolveEdges_XPBD(ClothMeshState s, float tildedCompliance)
        {
            for (int e = 0; e < s.Edges.Length; e += 2)
            {
                int i = s.Edges[e], j = s.Edges[e + 1];
                float wi = s.IsPinned[i] ? 0f : s.InvMass[i];
                float wj = s.IsPinned[j] ? 0f : s.InvMass[j];
                float wSum = wi + wj + tildedCompliance;
                if (wSum < 1e-10f) continue;

                Vector3 d   = s.PredPosition[i] - s.PredPosition[j];
                float   len = d.magnitude;
                if (len < 1e-7f) continue;

                float   C      = len - s.RestEdgeLen[e / 2];
                float   lambda = -C / wSum;
                Vector3 corr   = (d / len) * lambda;

                if (!s.IsPinned[i]) s.PredPosition[i] += wi * corr;
                if (!s.IsPinned[j]) s.PredPosition[j] -= wj * corr;
            }
        }

        private void ApplyClothToClothCollision(ClothMeshState s)
        {
            float twoT   = s.Params.Thickness * 2f;
            float twoTSq = twoT * twoT;

            foreach (ClothMeshState other in activeStates)
            {
                if (other == s || !other.Params.ClothToCloth) continue;
                if (!s.WorldBounds.Intersects(other.WorldBounds)) continue;

                for (int i = 0; i < s.VertCount; i++)
                {
                    if (s.IsPinned[i]) continue;
                    Vector3 pi = s.Position[i];

                    for (int j = 0; j < other.VertCount; j++)
                    {
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
            Transform tf = s.Renderer != null ? s.Renderer.transform : transform;
            for (int i = 0; i < s.VertCount; i++)
                s.LocalVerts[i] = tf.InverseTransformPoint(s.Position[i]);
            s.WorkMesh.vertices = s.LocalVerts;
            s.WorkMesh.RecalculateBounds();   // CRITICAL: without this Unity frustum-culls the mesh
            s.WorkMesh.RecalculateNormals();
            s.WorkMesh.RecalculateTangents();

            // Force collider to pick up changed vertex positions.
            if (s.ClothCollider != null)
            {
                s.ClothCollider.sharedMesh = null;
                s.ClothCollider.sharedMesh = s.WorkMesh;
            }
        }
        private static Vector3 ComputePinCentroid(ClothMeshState s)
        {
            if (s.PinData != null && s.PinData.Length > 0)
            {
                Vector3 sum = Vector3.zero;
                foreach (var pd in s.PinData) sum += s.Position[pd.VertexIndex];
                return sum / s.PinData.Length;
            }
            if (s.FallbackPinLocalPos != null)
            {
                Vector3 sum = Vector3.zero; int cnt = 0;
                for (int i = 0; i < s.VertCount; i++)
                    if (s.IsPinned[i]) { sum += s.Position[i]; cnt++; }
                return cnt > 0 ? sum / cnt : Vector3.zero;
            }
            return Vector3.zero;
        }
        private static void ApplyHardPins(ClothMeshState s, Vector3[] dst)
        {
            // Standard bone-weight pins
            foreach (var pd in s.PinData)
                dst[pd.VertexIndex] = ComputePinWorldPos(s, pd);

            // Fallback root-relative pins (set when mesh has no usable boneWeights)
            if (s.FallbackPinLocalPos != null && s.FallbackPinRoot != null)
                for (int i = 0; i < s.VertCount; i++)
                    if (s.IsPinned[i])
                        dst[i] = s.FallbackPinRoot.TransformPoint(s.FallbackPinLocalPos[i]);
        }
        private static void UpdatePins(ClothMeshState s) => ApplyHardPins(s, s.Position);

        private static Vector3 ComputePinWorldPos(ClothMeshState s, ClothBonePinData pd)
        {
            Vector3 rp = pd.RestPos;
            Vector4 rh = new Vector4(rp.x, rp.y, rp.z, 1f);
            Vector4 sum = Vector4.zero;
            for (int k = 0; k < 4; k++)
            {
                float w = pd.BoneWeights[k];
                int   b = pd.BoneIndices[k];
                if (w < 0.0001f || b < 0 || b >= s.PinBoneArray.Length) continue;
                if (s.PinBoneArray[b] == null) continue;
                Matrix4x4 m = s.PinBoneArray[b].localToWorldMatrix * s.PinBindPoses[b];
                sum += w * (m * rh);
            }
            return new Vector3(sum.x, sum.y, sum.z);
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
            Vector3[] localRestFinal = new Vector3[state.VertCount];
            for (int i = 0; i < state.VertCount; i++)
                localRestFinal[i] = tf.InverseTransformPoint(worldRest[i]);
            state.WorkMesh.vertices = localRestFinal;
            state.WorkMesh.triangles = tris;
            state.WorkMesh.uv = weldedUvs;
            state.WorkMesh.RecalculateBounds();
            state.WorkMesh.RecalculateNormals();
            BoneWeight[] bw         = src.boneWeights;
            Vector3[]    bindVerts  = src.vertices;  // bind-pose vertices for RestPos
            BoneWeight[] remappedBw = new BoneWeight[state.VertCount];
            Vector3[] remappedBindVerts = new Vector3[state.VertCount];
            for (int i = 0; i < state.VertCount; i++)
            {
                int srcIdx = newToOriginalMap[i];
                if (bw != null && srcIdx >= 0 && srcIdx < bw.Length)
                    remappedBw[i] = bw[srcIdx];
                if (bindVerts != null && srcIdx >= 0 && srcIdx < bindVerts.Length)
                    remappedBindVerts[i] = bindVerts[srcIdx];
            }
            IdentifyPins(state, smr, remappedBw, remappedBindVerts, worldRest);

            // Store rest positions in character-root local space for the Compression constraint.
            if (chaCtrl != null)
            {
                state.RestBodyLocalPos = new Vector3[state.VertCount];
                for (int i = 0; i < state.VertCount; i++)
                    state.RestBodyLocalPos[i] = chaCtrl.transform.InverseTransformPoint(worldRest[i]);
            }

            BuildEdges(state, tris, worldRest);
            float totalMass = 1.0f;
            float mass      = totalMass / Mathf.Max(1, state.VertCount);
            for (int i = 0; i < state.VertCount; i++)
            {
                state.Mass[i]    = mass;
                state.InvMass[i] = state.IsPinned[i] ? 0f : 1f / mass;
            }

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
            UpdatePins(state);
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
        }

        private static void AddEdge(int a, int b, HashSet<long> set, List<int> list, List<float> lens, Vector3[] pos)
        {
            int lo = Mathf.Min(a, b), hi = Mathf.Max(a, b);
            long key = (long)lo << 32 | (uint)hi;
            if (!set.Add(key)) return;
            list.Add(lo); list.Add(hi);
            lens.Add((pos[lo] - pos[hi]).magnitude);
        }

        private void IdentifyPins(ClothMeshState s, SkinnedMeshRenderer smr, BoneWeight[] bw, Vector3[] bindVerts, Vector3[] worldRest)
        {
            const float PinThreshold = 0.75f;

            var usedBones = new HashSet<int>();
            if (bw != null)
            {
                for (int i = 0; i < s.VertCount && i < bw.Length; i++)
                {
                    float dom = Mathf.Max(Mathf.Max(bw[i].weight0, bw[i].weight1),
                                         Mathf.Max(bw[i].weight2, bw[i].weight3));
                    if (dom >= PinThreshold)
                    {
                        s.IsPinned[i] = true;
                        if (bw[i].weight0 > 0.001f) usedBones.Add(bw[i].boneIndex0);
                        if (bw[i].weight1 > 0.001f) usedBones.Add(bw[i].boneIndex1);
                        if (bw[i].weight2 > 0.001f) usedBones.Add(bw[i].boneIndex2);
                        if (bw[i].weight3 > 0.001f) usedBones.Add(bw[i].boneIndex3);
                    }
                }
            }

            int maxOrig = 0;
            foreach (int bi in usedBones) if (bi > maxOrig) maxOrig = bi;
            Transform[]  bones     = smr.bones;
            Matrix4x4[]  bindPoses = smr.sharedMesh.bindposes;

            s.PinBoneArray = new Transform[maxOrig + 1];
            s.PinBindPoses = new Matrix4x4[maxOrig + 1];
            foreach (int bi in usedBones)
            {
                s.PinBoneArray[bi] = (bones != null && bi < bones.Length) ? bones[bi] : null;
                s.PinBindPoses[bi] = (bindPoses != null && bi < bindPoses.Length) ? bindPoses[bi] : Matrix4x4.identity;
            }

            var pins = new List<ClothBonePinData>();
            if (bw != null)
            {
                for (int i = 0; i < s.VertCount && i < bw.Length; i++)
                {
                    if (!s.IsPinned[i]) continue;
                    Vector3 restBindPos = (bindVerts != null && i < bindVerts.Length) ? bindVerts[i] : Vector3.zero;
                    pins.Add(new ClothBonePinData
                    {
                        VertexIndex = i,
                        BoneIndices  = new int[]  { bw[i].boneIndex0, bw[i].boneIndex1, bw[i].boneIndex2, bw[i].boneIndex3 },
                        BoneWeights  = new float[]{ bw[i].weight0,    bw[i].weight1,    bw[i].weight2,    bw[i].weight3    },
                        RestPos      = restBindPos
                    });
                }
            }
            s.PinData = pins.ToArray();

            if (s.PinData.Length == 0 && chaCtrl != null)
            {
                float yMin = float.MaxValue, yMax = float.MinValue;
                for (int i = 0; i < s.VertCount; i++)
                {
                    if (worldRest[i].y < yMin) yMin = worldRest[i].y;
                    if (worldRest[i].y > yMax) yMax = worldRest[i].y;
                }
                float yThresh = yMin + (yMax - yMin) * 0.90f; // top 10 %

                s.FallbackPinLocalPos = new Vector3[s.VertCount];
                s.FallbackPinRoot     = chaCtrl.transform;
                for (int i = 0; i < s.VertCount; i++)
                {
                    if (worldRest[i].y >= yThresh)
                    {
                        s.IsPinned[i]             = true;
                        s.FallbackPinLocalPos[i]  = chaCtrl.transform.InverseTransformPoint(worldRest[i]);
                    }
                }
            }
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
                        });
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
    }
}
