using System;
using System.Collections.Generic;
using UnityEngine;

namespace StudioModsMSG
{
    // -----------------------------------------------------------------------
    // Per-mesh simulation mode
    // -----------------------------------------------------------------------
    enum ClothSimulationMode
    {
        /// <summary>Always simulating every frame.</summary>
        Continuous        = 0,
        /// <summary>
        /// Cloth is frozen until the user holds the ManualDeform keybinding and
        /// right-click-drags near the mesh.  While the binding is held, vertices
        /// within DeformRadius of the cursor are awakened and pulled toward the
        /// cursor position, creating physics-based cloth sculpting.  On key
        /// release the simulation pauses automatically.
        /// </summary>
        ManualDeformation = 1,
    }

    // -----------------------------------------------------------------------
    // Per-cloth-mesh physics parameters (mutable at runtime via GUI sliders)
    // -----------------------------------------------------------------------
    class ClothPhysicsParams
    {
        // xPBD compliance-based stiffness.  α̃ = 1/(k·dt²), lower k = stretchier cloth.
        // Defaults tuned for realistic underwear/bra: stiff fabric that barely stretches,
        // high damping so it settles immediately when the character stops moving.
        public float StretchStiffness = 4000f;  // stiff — fabric resists stretching (bra / underwear)
        public float BendStiffness    = 4f;     // moderate fold resistance
        public float Damping          = 8f;     // high damping → settles quickly, no oscillation at rest
        public float Thickness        = 0.012f; // thin collision offset (metres)
        public float RestInflate      = 0.006f; // minimal inflate to prevent z-fighting
        public float Gravity          = -9.81f; // Y gravity (game units/s²; HS2 ≈ 1 unit = 1 m)
        public float Weight           = 0.55f;  // light cloth — underwear is thin fabric
        // Quality presets: 0.25 / 0.5 / 0.75 / 1.0 / 2.0 / 4.0 / 6.0 / 8.0
        // Runtime maps these multipliers to concrete solver counts.
        public float Substeps         = 2f;
        public float Iterations       = 2f;
        public bool  ClothToCloth     = true;   // enable inter-cloth collision
        // 0 = off; 1 = maximum.  Scales down the rest-edge target lengths so the
        // fabric tries to shrink.  Body colliders prevent penetration, so the cloth
        // presses inward and tightens against the body instead of ballooning.
        // Does NOT freeze motion — it applies outward pressure against the body.
        public float Compression      = 0f;
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

        // -- Toggle & per-mesh simulation control --
        public bool IsActive          = false;
        public bool SimulationPaused  = false;  // independent pause/play per mesh

        // -- Simulation mode --
        public ClothSimulationMode SimulationMode = ClothSimulationMode.Continuous;
        /// <summary>
        /// World-space radius around the cursor used in ManualDeformation mode:
        /// vertices within this radius are awakened and attracted toward the cursor.
        /// </summary>
        public float  DeformRadius = 0.12f;
        /// <summary>Frames a vertex stays awake after the last manual-deform interaction.</summary>
        public const int TriggerCooldownFrames = 60;
        /// <summary>Per-vertex countdown; 0 = frozen, > 0 = simulating (allocated on activation).</summary>
        public int[]  TriggerCooldown;

        // -- Manual-deformation drag state (set each frame by ClothSoftBodyRuntime) --
        /// <summary>Closest mesh vertex to the cursor, or -1 when not dragging.</summary>
        public int     DragVertexIndex   = -1;
        /// <summary>World-space target position the dragged vertex is attracted toward.</summary>
        public Vector3 DragTargetWorld   = Vector3.zero;
        /// <summary>True while the user is actively holding the keybind and right-click dragging.</summary>
        public bool    IsDragging        = false;

        // -- Per-mesh LBS cache (fixes cross-mesh bone-follow corruption) --
        // Must NOT be shared across states; each state owns its own buffer.
        public Vector3[] SkinnedPositions;      // current-frame LBS output for every vertex
        public Vector3[] PrevSkinnedPositions;  // previous-frame LBS output
        // Counts down from WarmupDuration on activation; simulation uses extra damping
        // during this window to prevent the init-pop when colliders fire all at once.
        public int WarmupFramesRemaining = 0;
        public const int WarmupDuration = 12;

        // -- Topology (built once on activation, remains frozen) --
        public int     VertCount;
        public int[]   Tris;            // flat [a,b,c, a,b,c, ...] triangle list
        public int[]   Edges;           // flat [i,j, i,j, ...] deduplicated edges
        public float[] RestEdgeLen;     // rest length for each edge pair
        public int[][] EdgeColorGroups; // graph-colored edge groups: each sub-array is a flat [i,j,i,j,...] batch with no shared vertices
        public int[]   BendPairs;       // flat [i,j, i,j, ...] opposite verts across shared edges
        public float[] RestBendLen;     // rest length for each bend pair
        public int[][] BendColorGroups; // graph-colored bend groups (same contract as EdgeColorGroups)

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
        public float[]     PinBlend;                // per-vertex: 0=fully simulated, 1=fully skinned (pinned)
        public Vector3[]   SrcBindVerts;            // bind-pose mesh-local vertices (per welded vertex, for per-frame LBS)
        public Matrix4x4[] BindPoses;               // src.bindposes (bone → mesh-local inverse)

        // -- Debug visualization --
        public bool   ShowPinBoneGizmos = false;
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
        public Matrix4x4    PrevRendererLocalToWorld = Matrix4x4.identity;
        public bool         HasPrevRendererTransform = false;

        // -- Broadphase --
        public Bounds WorldBounds;

        public long VBDHandle = 0;

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
