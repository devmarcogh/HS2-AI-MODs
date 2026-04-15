using UnityEngine;
using System.Collections.Generic;
using System.Threading.Tasks;

namespace StudioModsMSG
{
    public class BodyProxyCollider : MonoBehaviour
    {
        [Header("Configuración")]
        [Tooltip("Si es 1, usa todos los vértices. Si es 2, usa la mitad, etc. Ayuda al rendimiento.")]
        public int decimationFactor = 4; 
        public float bodyParticleRadius = 0.03f; // Radio virtual de cada vértice del cuerpo

        // Datos cacheados (Cero Garbage Collection en Update)
        private Vector3[] bindPosesLocal;
        private BoneWeight[] boneWeights;
        public Vector3[] AnimatedPositions { get; private set; }
        public int ProxyCount { get; private set; }

        private Transform[] bones;
        private Matrix4x4[] bindPoses;
        private Matrix4x4[] boneMatrices;

        /// <summary>
        /// Initialises the proxy. If <paramref name="boneWhitelist"/> is provided, only
        /// vertices whose dominant bone is in the whitelist are included. This is used
        /// to restrict proxy particles to a specific body region (e.g. breasts only).
        /// </summary>
        public void Initialize(SkinnedMeshRenderer smr, HashSet<string> boneWhitelist = null)
        {
            Mesh mesh = smr.sharedMesh;
            Vector3[]    originalVerts   = mesh.vertices;
            BoneWeight[] originalWeights = mesh.boneWeights;
            Transform[]  smrBones        = smr.bones;

            bones        = smrBones;
            bindPoses    = mesh.bindposes;
            boneMatrices = new Matrix4x4[bones.Length];

            // 1) Build candidate list with full-resolution filtering by dominant bone.
            var candidateIndices = new List<int>(originalVerts.Length);
            for (int i = 0; i < originalVerts.Length; i++)
            {
                if (boneWhitelist != null)
                {
                    BoneWeight bw = originalWeights[i];
                    int   dom = bw.boneIndex0; float dw = bw.weight0;
                    if (bw.weight1 > dw) { dw = bw.weight1; dom = bw.boneIndex1; }
                    if (bw.weight2 > dw) { dw = bw.weight2; dom = bw.boneIndex2; }
                    if (bw.weight3 > dw) {                   dom = bw.boneIndex3; }

                    if (dom < 0 || dom >= smrBones.Length || smrBones[dom] == null ||
                        !boneWhitelist.Contains(smrBones[dom].name))
                        continue;
                }
                candidateIndices.Add(i);
            }

            // 2) Decimate AFTER filtering so small regions (breasts/belly) keep enough particles.
            int stride = Mathf.Max(1, decimationFactor);
            var includedIndices = new List<int>(candidateIndices.Count / stride + 1);
            for (int i = 0; i < candidateIndices.Count; i += stride)
                includedIndices.Add(candidateIndices[i]);

            if (includedIndices.Count == 0 && candidateIndices.Count > 0)
            {
                // Keep at least one sample when candidate set exists.
                includedIndices.Add(candidateIndices[0]);
            }

            ProxyCount        = includedIndices.Count;
            bindPosesLocal    = new Vector3[ProxyCount];
            boneWeights       = new BoneWeight[ProxyCount];
            AnimatedPositions = new Vector3[ProxyCount];

            for (int j = 0; j < includedIndices.Count; j++)
            {
                int src = includedIndices[j];
                bindPosesLocal[j] = originalVerts[src];
                boneWeights[j]    = originalWeights[src];
            }
        }

        // Ejecutar ANTES del SimulateStep de la tela
        public void UpdateProxyPositions()
        {
            if (ProxyCount == 0) return;

            // 1. Precalcular las matrices de deformación de los huesos (main thread — Transform reads)
            for (int i = 0; i < bones.Length; i++)
            {
                if (bones[i] == null) continue;
                boneMatrices[i] = bones[i].localToWorldMatrix * bindPoses[i];
            }

            // 2. Linear Blend Skinning (LBS) — parallel when enough particles to justify overhead
            if (ProxyCount > 128)
            {
                // Capture refs for closure (no shared mutable state)
                Vector3[]    localBindPos = bindPosesLocal;
                BoneWeight[] localBW      = boneWeights;
                Matrix4x4[]  localBM      = boneMatrices;
                Vector3[]    outPos       = AnimatedPositions;

                Parallel.For(0, ProxyCount, i =>
                {
                    Vector3 p = localBindPos[i];
                    BoneWeight bw = localBW[i];
                    Vector3 worldPos = Vector3.zero;

                    if (bw.weight0 > 0) worldPos += localBM[bw.boneIndex0].MultiplyPoint3x4(p) * bw.weight0;
                    if (bw.weight1 > 0) worldPos += localBM[bw.boneIndex1].MultiplyPoint3x4(p) * bw.weight1;
                    if (bw.weight2 > 0) worldPos += localBM[bw.boneIndex2].MultiplyPoint3x4(p) * bw.weight2;
                    if (bw.weight3 > 0) worldPos += localBM[bw.boneIndex3].MultiplyPoint3x4(p) * bw.weight3;

                    outPos[i] = worldPos;
                });
            }
            else
            {
                for (int i = 0; i < ProxyCount; i++)
                {
                    Vector3 p = bindPosesLocal[i];
                    BoneWeight bw = boneWeights[i];
                    Vector3 worldPos = Vector3.zero;

                    if (bw.weight0 > 0) worldPos += boneMatrices[bw.boneIndex0].MultiplyPoint3x4(p) * bw.weight0;
                    if (bw.weight1 > 0) worldPos += boneMatrices[bw.boneIndex1].MultiplyPoint3x4(p) * bw.weight1;
                    if (bw.weight2 > 0) worldPos += boneMatrices[bw.boneIndex2].MultiplyPoint3x4(p) * bw.weight2;
                    if (bw.weight3 > 0) worldPos += boneMatrices[bw.boneIndex3].MultiplyPoint3x4(p) * bw.weight3;

                    AnimatedPositions[i] = worldPos;
                }
            }
        }
    }
}