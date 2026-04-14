using UnityEngine;
using System.Collections.Generic;

namespace StudioModsMSG
{
    public class BodyProxyCollider : MonoBehaviour
    {
        [Header("Configuración")]
        [Tooltip("Si es 1, usa todos los vértices. Si es 2, usa la mitad, etc. Ayuda al rendimiento.")]
        public int decimationFactor = 4; 
        public float bodyParticleRadius = 0.015f; // Radio virtual de cada vértice del cuerpo

        // Datos cacheados (Cero Garbage Collection en Update)
        private Vector3[] bindPosesLocal;
        private BoneWeight[] boneWeights;
        public Vector3[] AnimatedPositions { get; private set; }
        public int ProxyCount { get; private set; }

        private Transform[] bones;
        private Matrix4x4[] bindPoses;
        private Matrix4x4[] boneMatrices;

        public void Initialize(SkinnedMeshRenderer smr)
        {
            Mesh mesh = smr.sharedMesh;
            Vector3[] originalVerts = mesh.vertices;
            BoneWeight[] originalWeights = mesh.boneWeights;

            bones = smr.bones;
            bindPoses = mesh.bindposes;
            boneMatrices = new Matrix4x4[bones.Length];

            // Diezmado: Tomamos solo 1 de cada N vértices para ahorrar cálculos
            ProxyCount = originalVerts.Length / decimationFactor;
            
            bindPosesLocal = new Vector3[ProxyCount];
            boneWeights = new BoneWeight[ProxyCount];
            AnimatedPositions = new Vector3[ProxyCount];

            int proxyIndex = 0;
            for (int i = 0; i < originalVerts.Length; i += decimationFactor)
            {
                if (proxyIndex >= ProxyCount) break;
                bindPosesLocal[proxyIndex] = originalVerts[i];
                boneWeights[proxyIndex] = originalWeights[i];
                proxyIndex++;
            }
        }

        // Ejecutar ANTES del SimulateStep de la tela
        public void UpdateProxyPositions()
        {
            if (ProxyCount == 0) return;

            // 1. Precalcular las matrices de deformación de los huesos
            for (int i = 0; i < bones.Length; i++)
            {
                // Multiplicamos la matriz del hueso en el mundo por su pose de descanso (BindPose)
                boneMatrices[i] = bones[i].localToWorldMatrix * bindPoses[i];
            }

            // 2. Linear Blend Skinning (LBS) Manual
            for (int i = 0; i < ProxyCount; i++)
            {
                Vector3 p = bindPosesLocal[i];
                BoneWeight bw = boneWeights[i];
                Vector3 worldPos = Vector3.zero;

                // Aplicar la influencia de hasta 4 huesos por vértice
                if (bw.weight0 > 0) worldPos += boneMatrices[bw.boneIndex0].MultiplyPoint3x4(p) * bw.weight0;
                if (bw.weight1 > 0) worldPos += boneMatrices[bw.boneIndex1].MultiplyPoint3x4(p) * bw.weight1;
                if (bw.weight2 > 0) worldPos += boneMatrices[bw.boneIndex2].MultiplyPoint3x4(p) * bw.weight2;
                if (bw.weight3 > 0) worldPos += boneMatrices[bw.boneIndex3].MultiplyPoint3x4(p) * bw.weight3;

                AnimatedPositions[i] = worldPos;
            }
        }
    }
}