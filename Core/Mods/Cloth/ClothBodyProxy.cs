using System;
using System.Collections.Generic;
using UnityEngine;

namespace StudioModsMSG
{
    /// <summary>
    /// Lightweight proxy of the body surface mesh, updated each frame by
    /// manually skinning ~120 selected vertices through bone transforms
    /// (no BakeMesh call — costs about 0.05 ms/frame).
    ///
    /// The proxy is used by ClothSoftBodyRuntime as a surface collider so
    /// simulated cloth verts are pushed out of the body geometry.
    /// </summary>
    sealed class ClothBodyProxy
    {
        // ------------------------------------------------------------------ //
        // Constants
        // ------------------------------------------------------------------ //
        public const int DefaultProxyVerts = 400;
        private const int LeafTriMax       = 8;
        private const int MaxBVHDepth      = 8;

        // ------------------------------------------------------------------ //
        // State
        // ------------------------------------------------------------------ //
        public bool IsReady => proxyVerts != null && proxyVerts.Length > 0 && bvhNodes != null;

        // -- Skinning data (frozen after Init) --
        private struct SkinVert
        {
            public Vector3 RestPos;
            public int[]   BoneIdx;      // up to 4, remapped to proxyBones
            public float[] BoneWeight;
        }
        private struct ProxyBone
        {
            public Transform Bone;
            public Matrix4x4 BindPose;   // inverse bind pose from renderer
        }

        private SkinVert[] proxyVerts;
        private int[]      proxyTris;     // flat triangle index list
        private ProxyBone[] proxyBones;

        // -- Per-frame buffers --
        private Vector3[] worldPos;

        // -- BVH --
        private struct BVHNode
        {
            public Bounds Bounds;
            public int    TriStart, TriCount;    // >0 = leaf
            public int    ChildLeft, ChildRight; // -1 = leaf
        }
        private BVHNode[] bvhNodes;
        private int[]     leafTriIdx;            // reordered triangle indices for BVH leaves

        // ------------------------------------------------------------------ //
        // Hit result
        // ------------------------------------------------------------------ //
        public struct HitResult
        {
            public bool    Hit;
            public Vector3 ClosestPoint;
            public Vector3 Normal;
            public float   Penetration;  // > 0 = inside body
        }

        // ================================================================== //
        // Init
        // ================================================================== //
        public bool Init(SkinnedMeshRenderer bodyRenderer, int maxVerts = DefaultProxyVerts)
        {
            if (bodyRenderer == null || bodyRenderer.sharedMesh == null) return false;

            Mesh         mesh      = bodyRenderer.sharedMesh;
            Vector3[]    allVerts  = mesh.vertices;
            int[]        allTris   = mesh.triangles;
            BoneWeight[] allWeights = mesh.boneWeights;
            Matrix4x4[]  bindPoses = mesh.bindposes;
            Transform[]  bones     = bodyRenderer.bones;

            if (allVerts == null || allVerts.Length == 0 ||
                allTris  == null || allTris.Length  == 0 ||
                allWeights == null || allWeights.Length == 0 ||
                bones == null || bones.Length == 0) return false;

            // 1. Select proxy vertex indices via greedy farthest-point sampling
            int n = Mathf.Min(maxVerts, allVerts.Length);
            int[] selected = FarthestPointSample(allVerts, n);

            // Map full-mesh index → proxy index (-1 if not selected)
            int[] f2p = new int[allVerts.Length];
            for (int i = 0; i < f2p.Length; i++) f2p[i] = -1;
            for (int p = 0; p < selected.Length; p++) f2p[selected[p]] = p;

            // 2. Keep proxy triangles where all 3 verts are selected
            var triList = new List<int>(allTris.Length / 4);
            for (int t = 0; t < allTris.Length; t += 3)
            {
                int a = f2p[allTris[t]], b = f2p[allTris[t + 1]], c = f2p[allTris[t + 2]];
                if (a >= 0 && b >= 0 && c >= 0) { triList.Add(a); triList.Add(b); triList.Add(c); }
            }
            if (triList.Count == 0) return false;
            proxyTris = triList.ToArray();

            // 3. Collect deduplicated bones referenced by selected verts
            var usedBones = new HashSet<int>();
            foreach (int vi in selected)
            {
                BoneWeight bw = allWeights[vi];
                if (bw.weight0 > 0.001f) usedBones.Add(bw.boneIndex0);
                if (bw.weight1 > 0.001f) usedBones.Add(bw.boneIndex1);
                if (bw.weight2 > 0.001f) usedBones.Add(bw.boneIndex2);
                if (bw.weight3 > 0.001f) usedBones.Add(bw.boneIndex3);
            }

            int[] origIdxs = new int[usedBones.Count];
            int   pb = 0;
            foreach (int bi in usedBones) origIdxs[pb++] = bi;

            // Build orig → proxy bone remap
            int maxOrig = 0;
            foreach (int bi in origIdxs) if (bi > maxOrig) maxOrig = bi;
            int[] boneRemap = new int[maxOrig + 1];
            for (int i = 0; i < boneRemap.Length; i++) boneRemap[i] = -1;
            for (int i = 0; i < origIdxs.Length; i++) boneRemap[origIdxs[i]] = i;

            proxyBones = new ProxyBone[origIdxs.Length];
            for (int i = 0; i < origIdxs.Length; i++)
            {
                int oi = origIdxs[i];
                proxyBones[i] = new ProxyBone
                {
                    Bone     = (bones != null && oi < bones.Length) ? bones[oi] : null,
                    BindPose = (bindPoses != null && oi < bindPoses.Length) ? bindPoses[oi] : Matrix4x4.identity
                };
            }

            // 4. Build skin vert data
            proxyVerts = new SkinVert[selected.Length];
            for (int p = 0; p < selected.Length; p++)
            {
                int vi = selected[p];
                BoneWeight bw = allWeights[vi];
                SkinVert sv = new SkinVert
                {
                    RestPos    = allVerts[vi],
                    BoneIdx    = new int[]  { Remap(boneRemap, bw.boneIndex0), Remap(boneRemap, bw.boneIndex1),
                                              Remap(boneRemap, bw.boneIndex2), Remap(boneRemap, bw.boneIndex3) },
                    BoneWeight = new float[]{ bw.weight0, bw.weight1, bw.weight2, bw.weight3 }
                };
                proxyVerts[p] = sv;
            }

            worldPos   = new Vector3[proxyVerts.Length];
            leafTriIdx = new int[proxyTris.Length / 3];
            for (int i = 0; i < leafTriIdx.Length; i++) leafTriIdx[i] = i;

            BuildBVH();
            return true;
        }

        private static int Remap(int[] map, int idx)
        {
            return (idx >= 0 && idx < map.Length) ? map[idx] : -1;
        }

        // -- Greedy farthest-point decimation --
        private static int[] FarthestPointSample(Vector3[] verts, int count)
        {
            if (count >= verts.Length)
            {
                int[] all = new int[verts.Length];
                for (int i = 0; i < all.Length; i++) all[i] = i;
                return all;
            }
            float[] minDist = new float[verts.Length];
            for (int i = 0; i < minDist.Length; i++) minDist[i] = float.MaxValue;

            int[] result = new int[count];
            result[0] = 0;
            for (int s = 1; s < count; s++)
            {
                Vector3 last = verts[result[s - 1]];
                for (int i = 0; i < verts.Length; i++)
                {
                    float d = (verts[i] - last).sqrMagnitude;
                    if (d < minDist[i]) minDist[i] = d;
                }
                float best = -1f; int bestIdx = 0;
                for (int i = 0; i < verts.Length; i++)
                    if (minDist[i] > best) { best = minDist[i]; bestIdx = i; }
                result[s] = bestIdx;
            }
            return result;
        }

        // ================================================================== //
        // BVH build (top-down median split)
        // ================================================================== //
        private void BuildBVH()
        {
            var nodes = new List<BVHNode>();
            // Use rest positions for initial build
            for (int p = 0; p < proxyVerts.Length; p++) worldPos[p] = proxyVerts[p].RestPos;
            BuildNode(nodes, 0, leafTriIdx.Length, 0);
            bvhNodes = nodes.ToArray();
        }

        private int BuildNode(List<BVHNode> nodes, int start, int end, int depth)
        {
            int idx   = nodes.Count;
            nodes.Add(default);
            int count = end - start;

            BVHNode node = new BVHNode { TriStart = start, TriCount = 0, ChildLeft = -1, ChildRight = -1 };

            if (count <= LeafTriMax || depth >= MaxBVHDepth)
            {
                node.TriCount = count;
                node.Bounds   = ComputeNodeBounds(start, end);
                nodes[idx]    = node;
                return idx;
            }

            // Longest-axis median split.
            // Compute centroid bounds directly from TriCentre() — no separate array needed.
            Bounds cb   = new Bounds(TriCentre(leafTriIdx[start]), Vector3.zero);
            for (int i = start + 1; i < end; i++) cb.Encapsulate(TriCentre(leafTriIdx[i]));

            Vector3 ext  = cb.extents;
            int axis = ext.x >= ext.y && ext.x >= ext.z ? 0 : (ext.y >= ext.z ? 1 : 2);
            int mid  = start + count / 2;
            // Partial sort: leafTriIdx[start..mid) have centre[axis] <= median,
            // leafTriIdx[mid..end) have centre[axis] >= median.
            PartialSort(start, end, mid, axis);

            node.ChildLeft  = BuildNode(nodes, start, mid, depth + 1);
            node.ChildRight = BuildNode(nodes, mid,   end, depth + 1);
            node.Bounds     = new Bounds();
            node.Bounds.Encapsulate(nodes[node.ChildLeft ].Bounds);
            node.Bounds.Encapsulate(nodes[node.ChildRight].Bounds);
            nodes[idx] = node;
            return idx;
        }

        private Vector3 TriCentre(int ti) =>
            (worldPos[proxyTris[ti * 3]] + worldPos[proxyTris[ti * 3 + 1]] + worldPos[proxyTris[ti * 3 + 2]]) / 3f;

        private Bounds ComputeNodeBounds(int start, int end)
        {
            int fi = leafTriIdx[start];
            Bounds b = new Bounds(worldPos[proxyTris[fi * 3]], Vector3.zero);
            for (int i = start; i < end; i++)
            {
                int ti = leafTriIdx[i];
                b.Encapsulate(worldPos[proxyTris[ti * 3]]);
                b.Encapsulate(worldPos[proxyTris[ti * 3 + 1]]);
                b.Encapsulate(worldPos[proxyTris[ti * 3 + 2]]);
            }
            return b;
        }

        // Partial quickselect: ensures leafTriIdx[k] has the k-th smallest TriCentre()[axis].
        // Uses TriCentre() directly — no external centres array, no indexing bugs.
        private void PartialSort(int start, int end, int k, int axis)
        {
            while (start < end - 1)
            {
                // Median-of-three pivot
                int mid = (start + end) / 2;
                float pivot = TriCentre(leafTriIdx[mid])[axis];

                int lo = start, hi = end - 1, i = start;
                while (i <= hi)
                {
                    float v = TriCentre(leafTriIdx[i])[axis];
                    if      (v < pivot) { Swap(ref leafTriIdx[lo], ref leafTriIdx[i]); lo++; i++; }
                    else if (v > pivot) { Swap(ref leafTriIdx[i],  ref leafTriIdx[hi]); hi--; }
                    else                { i++; }
                }
                if      (k < lo)      end   = lo;
                else if (k > hi)      start = hi + 1;
                else                  break;
            }
        }

        private static void Swap(ref int a, ref int b) { int t = a; a = b; b = t; }

        // ================================================================== //
        // Per-frame update — manual bone skinning + BVH refit
        // ================================================================== //
        public void UpdateProxy()
        {
            if (!IsReady) return;

            // Cache bone matrices
            var boneMats = new Matrix4x4[proxyBones.Length];
            for (int i = 0; i < proxyBones.Length; i++)
                boneMats[i] = proxyBones[i].Bone != null
                    ? proxyBones[i].Bone.localToWorldMatrix * proxyBones[i].BindPose
                    : Matrix4x4.identity;

            // Skin vertices
            for (int p = 0; p < proxyVerts.Length; p++)
            {
                ref SkinVert sv = ref proxyVerts[p];
                Vector4 rh  = new Vector4(sv.RestPos.x, sv.RestPos.y, sv.RestPos.z, 1f);
                Vector4 sum = Vector4.zero;
                for (int k = 0; k < 4; k++)
                {
                    float w = sv.BoneWeight[k];
                    int   b = sv.BoneIdx[k];
                    if (w < 0.0001f || b < 0) continue;
                    sum += w * (boneMats[b] * rh);
                }
                worldPos[p] = new Vector3(sum.x, sum.y, sum.z);
            }

            // Refit BVH
            RefitNode(0);
        }

        private Bounds RefitNode(int ni)
        {
            BVHNode node = bvhNodes[ni];
            Bounds b;
            if (node.TriCount > 0)
            {
                b = ComputeNodeBounds(node.TriStart, node.TriStart + node.TriCount);
            }
            else
            {
                Bounds lb = RefitNode(node.ChildLeft);
                Bounds rb = RefitNode(node.ChildRight);
                b = lb; b.Encapsulate(rb);
            }
            node.Bounds = b;
            bvhNodes[ni] = node;
            return b;
        }

        // ================================================================== //
        // Collision query — returns penetration info for a world-space point
        // ================================================================== //
        public HitResult QueryPoint(Vector3 pos, float radius)
        {
            HitResult best = default;
            float bestPen = -1f;
            QueryNode(0, pos, radius, ref best, ref bestPen);
            return best;
        }

        private void QueryNode(int ni, Vector3 pos, float r, ref HitResult best, ref float bestPen)
        {
            BVHNode node = bvhNodes[ni];
            // Expand bounds by radius for conservative test
            Bounds exp = new Bounds(node.Bounds.center, node.Bounds.size + Vector3.one * (r * 2f));
            if (!exp.Contains(pos)) return;

            if (node.TriCount > 0)
            {
                for (int i = node.TriStart; i < node.TriStart + node.TriCount; i++)
                    TestTri(leafTriIdx[i], pos, r, ref best, ref bestPen);
            }
            else
            {
                QueryNode(node.ChildLeft,  pos, r, ref best, ref bestPen);
                QueryNode(node.ChildRight, pos, r, ref best, ref bestPen);
            }
        }

        private void TestTri(int ti, Vector3 pos, float r, ref HitResult best, ref float bestPen)
        {
            Vector3 a = worldPos[proxyTris[ti * 3]];
            Vector3 b = worldPos[proxyTris[ti * 3 + 1]];
            Vector3 c = worldPos[proxyTris[ti * 3 + 2]];

            Vector3 closest = ClosestPointOnTriangle(pos, a, b, c);
            Vector3 delta    = pos - closest;
            float dist       = delta.magnitude;
            float pen        = r - dist;

            if (pen > bestPen)
            {
                Vector3 normal;
                if (dist > 1e-6f)
                {
                    normal = delta / dist;
                }
                else
                {
                    Vector3 raw = Vector3.Cross(b - a, c - a);
                    if (raw.sqrMagnitude < 1e-12f) return;
                    normal = raw.normalized;
                }

                bestPen = pen;
                best = new HitResult
                {
                    Hit          = pen > 0f,
                    ClosestPoint = closest,
                    Normal       = normal,
                    Penetration  = pen
                };
            }
        }

        private static Vector3 ClosestPointOnTriangle(Vector3 p, Vector3 a, Vector3 b, Vector3 c)
        {
            Vector3 ab = b - a, ac = c - a, ap = p - a;
            float d1 = Vector3.Dot(ab, ap), d2 = Vector3.Dot(ac, ap);
            if (d1 <= 0f && d2 <= 0f) return a;

            Vector3 bp = p - b;
            float d3 = Vector3.Dot(ab, bp), d4 = Vector3.Dot(ac, bp);
            if (d3 >= 0f && d4 <= d3) return b;

            float vc = d1 * d4 - d3 * d2;
            if (vc <= 0f && d1 >= 0f && d3 <= 0f) return a + (d1 / (d1 - d3)) * ab;

            Vector3 cp = p - c;
            float d5 = Vector3.Dot(ab, cp), d6 = Vector3.Dot(ac, cp);
            if (d6 >= 0f && d5 <= d6) return c;

            float vb = d5 * d2 - d1 * d6;
            if (vb <= 0f && d2 >= 0f && d6 <= 0f) return a + (d2 / (d2 - d6)) * ac;

            float va   = d3 * d6 - d5 * d4;
            float denom = 1f / (va + vb + vc);
            float v = vb * denom, w = vc * denom;
            return a + v * ab + w * ac;
        }
    }
}
