using System;
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
        public float StretchStiffness = 120f;   // stronger defaults so cloth feels less rubbery
        public float BendStiffness    = 6f;     // simple bend resistance from shared-edge opposite verts
        public float Damping          = 5f;     // higher damping to reduce oscillation
        public float Thickness        = 0.025f; // body-collision offset (metres)
        public float Gravity          = -9.81f; // Y gravity (game units/s²; HS2 ≈ 1 unit = 1 m)
        public float Weight           = 1.35f;  // scales gravity/inertia feel without changing topology
        // Quality presets: 0.25 / 0.5 / 0.75 / 1.0
        // Runtime maps these multipliers to concrete solver counts.
        public float Substeps         = 1f;
        public float Iterations       = 1f;
        public bool  ClothToCloth     = true;   // enable inter-cloth collision
        // 0 = off; 1 = maximum.  Scales down the rest-edge target lengths so the
        // fabric tries to shrink.  Body colliders prevent penetration, so the cloth
        // presses inward and tightens against the body instead of ballooning.
        // Does NOT freeze motion — it applies outward pressure against the body.
        public float Compression      = 0.05f;
        public float Elasticity       = 0.05f;  // legacy/reserved; StretchStiffness is primary control
        public bool  NeedsTangents    = true;    // RecalculateTangents per frame (needed for normal-mapped materials)

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
        public int[]   BendPairs;       // flat [i,j, i,j, ...] opposite verts across shared edges
        public float[] RestBendLen;     // rest length for each bend pair

        // -- Per-vertex runtime (world space) --
        public Vector3[] Position;
        public Vector3[] Velocity;
        public Vector3[] PredPosition;  // symplectic Euler prediction
        public Vector3[] PrevPosition;  // position at start of substep (for velocity update)
        public bool[]    IsPinned;
        public float[]   Mass;          // per-vertex mass
        public float[]   InvMass;       // 0 for pinned vertices

        // -- Pinning by selected skin bones --
        public List<string> PinSourceBoneNames = new List<string>(); // user-selected source bone names
        public BoneWeight[] VertexBoneWeights;      // remapped to welded vertices
        public Transform[] SkinBones;               // smr.bones snapshot
        public Transform[] PinFollowBones;          // per-vertex follow transform (parent of dominant source bone)
        public Vector3[]   PinFollowLocalPos;       // pinned-vertex local pos in PinFollowBones[i] space

        // -- Debug visualization --
        public bool   ShowPinBoneGizmos = true;
        public string FocusBoneName;

            // bones the user has starred so they appear at the top of the selector
            public HashSet<string> PinFavoriteBoneNames = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            // bones that actually dominate ≥1 vertex in the mesh (pre-computed from smr.boneWeights at entry creation)
            public HashSet<string> BonesWithDominantVertices;

        // -- Rendering (set on activation, cleaned on deactivation) --
        public Mesh         WorkMesh;           // owned working copy of rest-pose mesh
        public MeshFilter   Filter;             // added to the clothing object
        public MeshRenderer MeshRend;           // added to the clothing object
        public MeshCollider ClothCollider;      // runtime collider rebuilt from WorkMesh
        public Material[]   OriginalMaterials;  // saved from SMR on disable
        public Vector3[]    LocalVerts;         // cached buffer for WriteMesh (no GC per frame)
        public Vector3[]    RestBodyLocalPos;   // rest positions in chaCtrl local space (Compression + pinning)

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
