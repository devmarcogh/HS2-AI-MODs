using System.Collections.Generic;
using UnityEngine;

namespace StudioModsMSG
{
    // -----------------------------------------------------------------------
    // Per-cloth-mesh physics parameters (mutable at runtime via GUI sliders)
    // -----------------------------------------------------------------------
    class ClothPhysicsParams
    {
        // xPBD compliance-based stiffness.  α̃ = 1/(k·dt²), lower k = stretchier cloth.
        public float StretchStiffness = 8f;     // much lower now that pins are disabled
        public float BendStiffness    = 0.05f;  // (reserved for future dihedral bending)
        public float Damping          = 3f;     // higher damping to reduce oscillation
        public float Thickness        = 0.025f; // body-collision offset (metres)
        public float Gravity          = -5.0f;  // Y gravity (game units/s²; HS2 ≈ 1 unit = 1 m)
        public int   Substeps         = 3;      // physics substeps per LateUpdate (more = stable)
        public int   Iterations       = 12;     // xPBD constraint iterations per substep (12 = tight constraints, reduce tearing)
        public bool  ClothToCloth     = true;   // enable inter-cloth collision
    }

    // -----------------------------------------------------------------------
    // Per-bone pin entry — used to recompute pin world positions each frame
    // via manual skinning (no BakeMesh needed for pinned verts)
    // -----------------------------------------------------------------------
    struct ClothBonePinData
    {
        public int       VertexIndex;
        public int[]     BoneIndices;  // up to 4
        public float[]   BoneWeights;  // matching weights
        public Vector3   RestPos;      // bind-pose local vertex position
    }

    // -----------------------------------------------------------------------
    // Full runtime state for a single cloth SkinnedMeshRenderer
    // -----------------------------------------------------------------------
    class ClothMeshState
    {
        // -- Identity --
        public string              CategoryId;     // "Top", "Bot", etc.
        public string              MeshName;       // SMR name
        public SkinnedMeshRenderer Renderer;       // original SMR
        public ClothPhysicsParams  Params = new ClothPhysicsParams();

        // -- Toggle--
        public bool IsActive = false;

        // -- Topology (built once on activation, remains frozen) --
        public int     VertCount;
        public int[]   Tris;            // flat [a,b,c, a,b,c, ...] triangle list
        public int[]   Edges;           // flat [i,j, i,j, ...] deduplicated edges
        public float[] RestEdgeLen;     // rest length for each edge pair

        // -- Per-vertex runtime (world space) --
        public Vector3[] Position;
        public Vector3[] Velocity;
        public Vector3[] PredPosition;  // symplectic Euler prediction
        public Vector3[] PrevPosition;  // position at start of substep (for velocity update)
        public bool[]    IsPinned;
        public float[]   Mass;          // per-vertex mass
        public float[]   InvMass;       // 0 for pinned vertices

        // -- Pin binding: bone-weight pins (standard; most cloth items) --
        public Transform[]        PinBoneArray;   // bone transforms referenced by pins
        public Matrix4x4[]        PinBindPoses;   // inverse-bind pose per bone slot
        public ClothBonePinData[] PinData;

        // -- Pin binding: fallback root-relative pins (when mesh has no usable bone weights) --
        // These verts are pinned to char root transform instead of individual bones.
        public Vector3[]  FallbackPinLocalPos;  // position in chaCtrl.transform local space
        public Transform  FallbackPinRoot;      // = chaCtrl.transform

        // -- Rendering (set on activation, cleaned on deactivation) --
        public Mesh         WorkMesh;           // owned working copy of rest-pose mesh
        public MeshFilter   Filter;             // added to the clothing object
        public MeshRenderer MeshRend;           // added to the clothing object
        public MeshCollider ClothCollider;      // runtime collider rebuilt from WorkMesh
        public Material[]   OriginalMaterials;  // saved from SMR on disable
        public Vector3[]    LocalVerts;         // cached buffer for WriteMesh (no GC per frame)

        // -- Broadphase --
        public Bounds WorldBounds;
    }

    // -----------------------------------------------------------------------
    // Represents one clothing slot (e.g. "Top") and all its meshes
    // -----------------------------------------------------------------------
    class ClothPhysicsEntry
    {
        public string                CategoryId;   // "Top", "Bot", etc.
        public string                DisplayName;  // human-readable
        public bool                  IsExpanded;   // GUI accordion state
        public List<ClothMeshState>  Meshes = new List<ClothMeshState>();
    }
}
