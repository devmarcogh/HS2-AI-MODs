using System;
using System.Collections.Generic;
using UnityEngine;

namespace StudioModsMSG
{
    /// <summary>
    /// Selects which collision source is active for the cloth solver.
    /// Only one source is evaluated per frame.
    /// </summary>
    public enum CollisionSourceMode
    {
        /// <summary>Mirror the existing DynamicBoneColliderBase components on the character.</summary>
        Manual = 0,
        /// <summary>Use the auto-generated capsules and/or LBS proxy particles.</summary>
        Auto   = 1,
    }

    /// <summary>
    /// Collider generation mode for each bone group.
    /// </summary>
    public enum ColliderMode
    {
        Off     = 0,
        Capsule = 1,
        Proxy   = 2,
    }

    /// <summary>
    /// Runtime state for one auto-generated capsule collider.
    /// All geometry is stored in mesh-vertex space so we can apply the
    /// standard LBS skinning matrix (bone.LTW * bindpose) at update time
    /// without storing a separate bind-space transform.
    /// </summary>
    public struct AutoCapsuleState
    {
        public Transform BoneTransform;   // the dominant bone at runtime
        public Matrix4x4 BindPose;        // mesh.bindposes[boneIndex]  (mesh-local → bind-bone-local)
        public Vector3   MeshLocalCenter; // capsule centre in mesh-vertex space
        public Vector3   MeshLocalAxis;   // capsule long-axis direction in mesh-vertex space (normalised)
        public float     HalfHeight;      // half-length of the capsule cylinder (mesh-vertex units ≈ world metres at bind-time)
        public float     Radius;          // capsule radius (mesh-vertex units)
    }

    /// <summary>
    /// Builds per-bone capsule colliders from the body SkinnedMeshRenderer skin weights.
    /// </summary>
    public static class AutoCapsuleBuilder
    {
        // Predefined bone groups with substring patterns for HS2 / AI Shoujo bone names.
        // Matching is case-insensitive substring search against the full bone name.
        //
        // Source: verified against PluginABMX.cs bone name list (cf_J_* / cf_hit_* / cf_N_*).
        public static readonly (string Name, string[] Patterns)[] BoneGroupDefs =
        {
            // Head, neck, and all face sub-bones (eyes, mouth, nose, ears, cheeks, etc.)
            ("Head",    new[] { "cf_j_head", "cf_j_neck",  "cf_j_face",}),
            /*  "cf_j_chin",
                                "cf_j_cheek","cf_j_mayu",  "cf_j_eye",   "cf_j_nose",
                                "cf_j_mouth","cf_j_mouthup","cf_j_mouthlow",
                                "cf_j_ear",  "cf_j_nosebr","cf_j_nose_tip" }),
            */
            // Spine, pelvis, hips, shoulders, glutes, and lower-torso detail bones.
            // Includes siri (ass), kokan (groin), ana, and skirt bones (cf_d_sk).
            ("Torso",   new[] { "cf_j_spine", "cf_j_kosi",  "cf_j_shoulder",
                                "cf_j_siri",  "cf_hit_siri","cf_j_kokan",
                                "cf_n_height"  }),

            // Upper arm, forearm, elbow (note: actual token is "elbo" not "elbow"), wrist, hand, fingers.
            ("Arms",    new[] { "cf_j_armup", "cf_j_armlow","cf_j_armelbo",
                                "cf_j_hand",  "cf_j_wrist"                }),

            // Full leg: upper/lower thigh, knee, calve, foot, toes. Ankle pattern kept as fallback.
            ("Legs",    new[] { "cf_j_legup", "cf_j_leglow","cf_j_legknee",
                                "cf_j_legupdam","cf_j_foot", "cf_j_toes",  "ankle" }),

            // Breast volume, nipple, and collision hit-bones. "mune" covers all cf_J_Mune* and cf_hit_Mune*.
            ("Breasts", new[] { "mune",        "cf_hit_mune","cf_j_mune_nip" }),
        };

        /// <summary>
        /// Returns all bone names from <paramref name="allBones"/> whose name matches
        /// any pattern in the named group.
        /// </summary>
        public static HashSet<string> GetBoneNamesForGroup(Transform[] allBones, string groupName)
        {
            var result = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            string[] patterns = null;

            for (int i = 0; i < BoneGroupDefs.Length; i++)
            {
                if (string.Equals(BoneGroupDefs[i].Name, groupName, StringComparison.OrdinalIgnoreCase))
                {
                    patterns = BoneGroupDefs[i].Patterns;
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
        /// Builds a list of <see cref="AutoCapsuleState"/> fitted to the bones
        /// in <paramref name="boneNames"/> by analysing the mesh skin weights.
        /// </summary>
        /// <param name="smr">Body SkinnedMeshRenderer.</param>
        /// <param name="boneNames">Set of bone names to generate capsules for.</param>
        /// <param name="minVertsPerBone">Minimum dominant-vertex count; bones below this are skipped.</param>
        /// <param name="maxCapsules">Hard cap on total capsule count.</param>
        public static List<AutoCapsuleState> Build(
            SkinnedMeshRenderer smr,
            HashSet<string>     boneNames,
            int                 minVertsPerBone = 8,
            int                 maxCapsules     = 50)
        {
            var result = new List<AutoCapsuleState>();
            if (smr == null || smr.sharedMesh == null || boneNames == null || boneNames.Count == 0)
                return result;

            Mesh       mesh      = smr.sharedMesh;
            Vector3[]  verts     = mesh.vertices;
            BoneWeight[] weights = mesh.boneWeights;
            Matrix4x4[]  bindPoses = mesh.bindposes;
            Transform[]  bones   = smr.bones;

            if (verts == null || weights == null || bindPoses == null || bones == null) return result;

            // Build name → index map
            var boneIndexByName = new Dictionary<string, int>(StringComparer.OrdinalIgnoreCase);
            for (int i = 0; i < bones.Length; i++)
                if (bones[i] != null && !boneIndexByName.ContainsKey(bones[i].name))
                    boneIndexByName[bones[i].name] = i;

            // Resolve target bone indices
            var targetBoneIndices = new HashSet<int>();
            foreach (string name in boneNames)
            {
                int idx;
                if (boneIndexByName.TryGetValue(name, out idx))
                    targetBoneIndices.Add(idx);
            }

            if (targetBoneIndices.Count == 0) return result;

            // Group mesh-local vertex positions by dominant bone
            var vertsByBone = new Dictionary<int, List<Vector3>>();
            for (int i = 0; i < weights.Length; i++)
            {
                BoneWeight bw = weights[i];
                int   dom = bw.boneIndex0; float dw = bw.weight0;
                if (bw.weight1 > dw) { dw = bw.weight1; dom = bw.boneIndex1; }
                if (bw.weight2 > dw) { dw = bw.weight2; dom = bw.boneIndex2; }
                if (bw.weight3 > dw) {                   dom = bw.boneIndex3; }

                if (!targetBoneIndices.Contains(dom)) continue;

                List<Vector3> bucket;
                if (!vertsByBone.TryGetValue(dom, out bucket))
                    vertsByBone[dom] = bucket = new List<Vector3>();
                bucket.Add(verts[i]);
            }

            int capsuleCount = 0;
            foreach (var kv in vertsByBone)
            {
                if (capsuleCount >= maxCapsules) break;

                int          boneIndex  = kv.Key;
                List<Vector3> meshVerts = kv.Value;

                if (meshVerts.Count < minVertsPerBone) continue;
                if (boneIndex < 0 || boneIndex >= bones.Length || bones[boneIndex] == null) continue;

                // Compute centroid in mesh-local space
                Vector3 center = Vector3.zero;
                for (int i = 0; i < meshVerts.Count; i++) center += meshVerts[i];
                center /= meshVerts.Count;

                // Compute extents along each world axis to find the principal direction
                float extX = 0f, extY = 0f, extZ = 0f;
                for (int i = 0; i < meshVerts.Count; i++)
                {
                    Vector3 d = meshVerts[i] - center;
                    float ax = Math.Abs(d.x), ay = Math.Abs(d.y), az = Math.Abs(d.z);
                    if (ax > extX) extX = ax;
                    if (ay > extY) extY = ay;
                    if (az > extZ) extZ = az;
                }

                Vector3 axis;
                float halfHeight, radius;
                if (extX >= extY && extX >= extZ)
                {
                    axis       = Vector3.right;
                    halfHeight = extX;
                    radius     = Mathf.Max(extY, extZ);
                }
                else if (extY >= extX && extY >= extZ)
                {
                    axis       = Vector3.up;
                    halfHeight = extY;
                    radius     = Mathf.Max(extX, extZ);
                }
                else
                {
                    axis       = Vector3.forward;
                    halfHeight = extZ;
                    radius     = Mathf.Max(extX, extY);
                }

                radius = Mathf.Max(0.005f, radius);   // enforce minimum

                result.Add(new AutoCapsuleState
                {
                    BoneTransform   = bones[boneIndex],
                    BindPose        = bindPoses[boneIndex],
                    MeshLocalCenter = center,
                    MeshLocalAxis   = axis,
                    HalfHeight      = halfHeight,
                    Radius          = radius,
                });
                capsuleCount++;
            }

            return result;
        }
    }
}
