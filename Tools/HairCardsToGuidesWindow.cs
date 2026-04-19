// =============================================================================
// HairCardsToGuidesWindow.cs   —   Unity Editor  (v2 — Full Auto Setup)
//
// INSTALLATION
//   Drop into  Assets/Editor/  in any Unity project (Editor-only assembly).
//   Open via:  Tools > Hair Cards to Guides
//
// WORKFLOW (one button)
//   1. Select the hair-cards mesh in the scene.
//   2. Open the tool, pick a physics preset, hit  [Full Auto Setup].
//
//   Under the hood it:
//     a) Builds a vertex-adjacency graph from triangles.
//     b) Flood-fills connected components  (1 component = 1 hair-card strip).
//     c) Double-BFS per component → diameter path = strip spine.
//     d) Laplacian-smooths the spine.
//     e) Orients root at top  (highest-Y first).
//     f) Creates  Strand_NNN / bone_00..N / bone_end  transform chains.
//     g) Optionally transfers SkinnedMeshRenderer bone weights.
//     h) Finds the character armature and auto-places sphere DynamicBone
//        colliders on head / neck / chest bones.
//     i) Adds and configures a DynamicBone component on every Strand root.
//
// DYNAMICBONE  (Unity Asset Store — third-party)
//   The tool detects DynamicBone at runtime via reflection so it compiles
//   whether or not the asset is imported.  A status indicator shows
//   "DynamicBone ✓ found" or "✗ not found" in the panel header.
//   Without DynamicBone the conversion still works; physics is just skipped.
//
// PHYSICS PRESETS
//   Soft   — long, flowing hair   (high damping, low stiffness)
//   Medium — typical mid-length hair
//   Stiff  — short or stiff hair  (low damping, high stiffness)
//   Custom — manual per-parameter control
// =============================================================================

using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using UnityEngine;
using UnityEditor;

public class HairCardsToGuidesWindow : EditorWindow
{
    // ══════════════════════════════════════════════════════════════════════
    //  Types
    // ══════════════════════════════════════════════════════════════════════

    private enum OutputMode    { TransformChain, GuideMesh, Both }
    private enum PhysicsPreset { Soft, Medium, Stiff, Custom }

    // ══════════════════════════════════════════════════════════════════════
    //  Settings
    // ══════════════════════════════════════════════════════════════════════

    // Source
    private GameObject _source;

    // Extraction
    private OutputMode _mode         = OutputMode.TransformChain;
    private int        _smoothIter   = 2;
    private float      _smoothStr    = 0.65f;
    private int        _minVerts     = 4;
    private bool       _addEndBone   = true;
    private bool       _xferWeights  = true;
    private bool       _hideOriginal = true;

    // Physics
    private bool          _applyPhysics = true;
    private PhysicsPreset _preset       = PhysicsPreset.Medium;
    private float         _damping      = 0.15f;
    private float         _elasticity   = 0.10f;
    private float         _stiffness    = 0.10f;
    private float         _inert        = 0.50f;
    private float         _radius       = 0.02f;
    private Vector3       _gravity      = new Vector3(0f, -0.02f, 0f);

    // Colliders
    private bool   _autoColliders    = true;
    private string _colliderKeywords = "head, neck, chest";
    private float  _colliderRadius   = 0.12f;

    // UI state
    private Vector2 _scroll;
    private bool    _showExtraction = true;
    private bool    _showPhysics    = true;
    private bool    _showColliders  = true;

    // ══════════════════════════════════════════════════════════════════════
    //  DynamicBone reflection cache
    //  Compiles whether or not the DynamicBone asset is in the project.
    // ══════════════════════════════════════════════════════════════════════

    private static Type s_DBType;
    private static Type s_DBColliderType;
    private static Type s_DBColliderBaseType;
    private static bool s_Searched;

    private static Type DBType
    {
        get { if (!s_Searched) FindDynamicBoneTypes(); return s_DBType; }
    }

    static void FindDynamicBoneTypes()
    {
        s_Searched = true;
        foreach (var asm in AppDomain.CurrentDomain.GetAssemblies())
        {
            try
            {
                foreach (var t in asm.GetTypes())
                {
                    if (t.Name == "DynamicBone")             s_DBType             = t;
                    if (t.Name == "DynamicBoneCollider")     s_DBColliderType     = t;
                    if (t.Name == "DynamicBoneColliderBase") s_DBColliderBaseType = t;
                }
            }
            catch { /* skip un-reflectable assemblies */ }
        }
    }

    static bool DynamicBoneAvailable => DBType != null;

    // ══════════════════════════════════════════════════════════════════════
    //  Window
    // ══════════════════════════════════════════════════════════════════════

    [MenuItem("Tools/Hair Cards to Guides")]
    static void Open() => GetWindow<HairCardsToGuidesWindow>("Hair Cards → Guides");

    void OnSelectionChange()
    {
        if (Selection.activeGameObject != null) _source = Selection.activeGameObject;
        Repaint();
    }

    // ══════════════════════════════════════════════════════════════════════
    //  GUI
    // ══════════════════════════════════════════════════════════════════════

    void OnGUI()
    {
        _scroll = EditorGUILayout.BeginScrollView(_scroll);
        EditorGUILayout.Space(4);

        // Header
        GUILayout.Label("Hair Cards  →  Guides  +  Physics", EditorStyles.largeLabel);
        DrawDBStatus();

        // ── Source ───────────────────────────────────────────────────────
        EditorGUILayout.Space(6);
        GUILayout.Label("Source", EditorStyles.boldLabel);
        _source = (GameObject)EditorGUILayout.ObjectField(
            "Hair Cards Object", _source, typeof(GameObject), allowSceneObjects: true);

        bool hasValidSource = SourceIsValid(out string srcError);
        if (_source != null && !hasValidSource)
            EditorGUILayout.HelpBox(srcError, MessageType.Warning);

        // ── Extraction settings ──────────────────────────────────────────
        EditorGUILayout.Space(4);
        _showExtraction = EditorGUILayout.Foldout(_showExtraction, "Extraction Settings", true);
        if (_showExtraction)
        {
            EditorGUI.indentLevel++;
            _mode         = (OutputMode)EditorGUILayout.EnumPopup("Output Mode",         _mode);
            _smoothIter   = EditorGUILayout.IntSlider("Smooth Iterations",  _smoothIter,  0, 10);
            _smoothStr    = EditorGUILayout.Slider(   "Smooth Strength",    _smoothStr,   0f, 1f);
            _minVerts     = EditorGUILayout.IntField( "Min Verts / Card",   _minVerts);
            _addEndBone   = EditorGUILayout.Toggle(   "Add End Bone",       _addEndBone);
            _xferWeights  = EditorGUILayout.Toggle(   "Transfer Weights",   _xferWeights);
            _hideOriginal = EditorGUILayout.Toggle(   "Hide Original",      _hideOriginal);
            EditorGUI.indentLevel--;
        }

        // ── Physics settings ─────────────────────────────────────────────
        EditorGUILayout.Space(4);
        _showPhysics = EditorGUILayout.Foldout(_showPhysics, "Physics (DynamicBone)", true);
        if (_showPhysics)
        {
            EditorGUI.indentLevel++;
            _applyPhysics = EditorGUILayout.Toggle("Apply DynamicBone", _applyPhysics);
            if (_applyPhysics)
            {
                EditorGUI.BeginChangeCheck();
                _preset = (PhysicsPreset)EditorGUILayout.EnumPopup("Preset", _preset);
                if (EditorGUI.EndChangeCheck() && _preset != PhysicsPreset.Custom)
                    ApplyPreset(_preset);

                using (new EditorGUI.DisabledScope(_preset != PhysicsPreset.Custom))
                {
                    _damping    = EditorGUILayout.Slider("Damping",    _damping,    0f, 1f);
                    _elasticity = EditorGUILayout.Slider("Elasticity", _elasticity, 0f, 1f);
                    _stiffness  = EditorGUILayout.Slider("Stiffness",  _stiffness,  0f, 1f);
                    _inert      = EditorGUILayout.Slider("Inertia",    _inert,      0f, 1f);
                    _radius     = EditorGUILayout.Slider("Radius",     _radius,     0f, 0.1f);
                    _gravity    = EditorGUILayout.Vector3Field("Gravity", _gravity);
                }

                if (!DynamicBoneAvailable)
                    EditorGUILayout.HelpBox(
                        "DynamicBone not found. Import it from the Asset Store " +
                        "or disable 'Apply DynamicBone' to skip physics.",
                        MessageType.Warning);
            }
            EditorGUI.indentLevel--;
        }

        // ── Collider settings ────────────────────────────────────────────
        if (_applyPhysics && _mode != OutputMode.GuideMesh)
        {
            EditorGUILayout.Space(4);
            _showColliders = EditorGUILayout.Foldout(_showColliders, "Colliders", true);
            if (_showColliders)
            {
                EditorGUI.indentLevel++;
                _autoColliders = EditorGUILayout.Toggle("Auto-place Colliders", _autoColliders);
                if (_autoColliders)
                {
                    _colliderKeywords = EditorGUILayout.TextField("Bone Keywords",   _colliderKeywords);
                    _colliderRadius   = EditorGUILayout.Slider(   "Collider Radius", _colliderRadius, 0.02f, 0.5f);
                    EditorGUILayout.HelpBox(
                        "Searches the character armature for bones whose names contain " +
                        "any keyword (comma-separated, case-insensitive). " +
                        "A sphere DynamicBoneCollider is added to each match.",
                        MessageType.None);
                }
                EditorGUI.indentLevel--;
            }
        }

        // ── Buttons ──────────────────────────────────────────────────────
        EditorGUILayout.Space(10);
        using (new EditorGUI.DisabledScope(!hasValidSource))
        {
            bool wantPhysics = _applyPhysics && _mode != OutputMode.GuideMesh;
            if (wantPhysics)
            {
                var prev = GUI.backgroundColor;
                GUI.backgroundColor = new Color(0.3f, 0.85f, 0.4f);
                if (GUILayout.Button("Full Auto Setup  (Convert + DynamicBone)", GUILayout.Height(38)))
                    Execute(applyDB: true);
                GUI.backgroundColor = prev;
            }
            if (GUILayout.Button("Convert Only  (no physics)", GUILayout.Height(30)))
                Execute(applyDB: false);
        }

        EditorGUILayout.EndScrollView();
    }

    void DrawDBStatus()
    {
        bool ok = DynamicBoneAvailable;
        var prev = GUI.contentColor;
        GUI.contentColor = ok ? new Color(0.4f, 1f, 0.5f) : new Color(1f, 0.45f, 0.35f);
        GUILayout.Label(ok ? "DynamicBone  ✓  found" : "DynamicBone  ✗  not found",
                        EditorStyles.miniLabel);
        GUI.contentColor = prev;
    }

    // ══════════════════════════════════════════════════════════════════════
    //  Physics presets  (tuned for anime / game hair)
    // ══════════════════════════════════════════════════════════════════════

    void ApplyPreset(PhysicsPreset p)
    {
        switch (p)
        {
            case PhysicsPreset.Soft:
                // Long flowing hair — lots of damping, very loose
                _damping = 0.25f; _elasticity = 0.05f; _stiffness = 0.05f;
                _inert   = 0.40f; _radius     = 0.02f;
                _gravity = new Vector3(0f, -0.04f, 0f);
                break;
            case PhysicsPreset.Medium:
                // Typical shoulder-length hair
                _damping = 0.15f; _elasticity = 0.10f; _stiffness = 0.10f;
                _inert   = 0.50f; _radius     = 0.02f;
                _gravity = new Vector3(0f, -0.02f, 0f);
                break;
            case PhysicsPreset.Stiff:
                // Short hair, pigtails, accessories
                _damping = 0.05f; _elasticity = 0.30f; _stiffness = 0.50f;
                _inert   = 0.60f; _radius     = 0.02f;
                _gravity = new Vector3(0f, -0.01f, 0f);
                break;
        }
    }

    // ══════════════════════════════════════════════════════════════════════
    //  Validation
    // ══════════════════════════════════════════════════════════════════════

    bool SourceIsValid(out string error)
    {
        if (_source == null) { error = "No object selected."; return false; }
        bool ok = _source.GetComponent<MeshFilter>()          != null
               || _source.GetComponent<SkinnedMeshRenderer>() != null;
        error = ok ? "" : "Object needs a MeshFilter or SkinnedMeshRenderer.";
        return ok;
    }

    // ══════════════════════════════════════════════════════════════════════
    //  Execute
    // ══════════════════════════════════════════════════════════════════════

    void Execute(bool applyDB)
    {
        var mf  = _source.GetComponent<MeshFilter>();
        var smr = _source.GetComponent<SkinnedMeshRenderer>();
        Mesh mesh = mf?.sharedMesh ?? smr?.sharedMesh;
        if (mesh == null) { Debug.LogError("[HairCards] Mesh is null."); return; }

        Vector3[] verts = mesh.vertices;
        int[]     tris  = mesh.triangles;

        // ── 1. Adjacency from triangles ───────────────────────────────────
        var adj = new Dictionary<int, HashSet<int>>(verts.Length);
        for (int i = 0; i < verts.Length; i++) adj[i] = new HashSet<int>();
        for (int i = 0; i < tris.Length; i += 3)
        {
            int a = tris[i], b = tris[i + 1], c = tris[i + 2];
            adj[a].Add(b); adj[b].Add(a);
            adj[a].Add(c); adj[c].Add(a);
            adj[b].Add(c); adj[c].Add(b);
        }

        // ── 2. Connected components (DFS) ─────────────────────────────────
        var visited    = new bool[verts.Length];
        var components = new List<List<int>>();
        for (int v = 0; v < verts.Length; v++)
        {
            if (visited[v]) continue;
            var comp  = new List<int>();
            var stack = new Stack<int>();
            stack.Push(v);
            while (stack.Count > 0)
            {
                int cur = stack.Pop();
                if (visited[cur]) continue;
                visited[cur] = true;
                comp.Add(cur);
                foreach (int nb in adj[cur])
                    if (!visited[nb]) stack.Push(nb);
            }
            components.Add(comp);
        }

        // ── 3. Per-component: spine + smooth + orient ─────────────────────
        var allWorldPts  = new List<Vector3>();
        var allOrigIdx   = new List<int>();
        var allEdges     = new List<(int a, int b)>();
        // strandRanges[i] = (startIndex, count) into allWorldPts
        var strandRanges = new List<(int start, int count)>();
        int offset = 0; int skipped = 0;
        var worldMat = _source.transform.localToWorldMatrix;

        foreach (var comp in components)
        {
            if (comp.Count < _minVerts) { skipped++; continue; }
            List<int> spine = FindLongestPath(adj, comp);
            if (spine.Count < 2)        { skipped++; continue; }

            var pts = spine.Select(i => worldMat.MultiplyPoint3x4(verts[i])).ToList();
            pts = SmoothPath(pts);
            EnsureRootAtTop(pts);

            int strandStart = offset;
            for (int i = 0; i < pts.Count - 1; i++)
                allEdges.Add((offset + i, offset + i + 1));
            allWorldPts.AddRange(pts);
            allOrigIdx.AddRange(spine);
            strandRanges.Add((strandStart, pts.Count));
            offset += pts.Count;
        }

        if (allWorldPts.Count == 0)
        {
            Debug.LogWarning($"[HairCards] No valid strands " +
                             $"(skipped {skipped} comps, minVerts={_minVerts}).");
            return;
        }

        // ── 4. Create hierarchy ───────────────────────────────────────────
        Undo.SetCurrentGroupName("Hair Cards to Guides");
        int undoGroup = Undo.GetCurrentGroup();

        var root = new GameObject(_source.name + "_Guides");
        Undo.RegisterCreatedObjectUndo(root, "Guides Root");
        root.transform.SetParent(_source.transform.parent, worldPositionStays: false);

        List<GameObject> strandRoots = null;

        if (_mode == OutputMode.GuideMesh || _mode == OutputMode.Both)
            BuildGuideMesh(root.transform, allWorldPts, allEdges, allOrigIdx, smr);

        if (_mode == OutputMode.TransformChain || _mode == OutputMode.Both)
            strandRoots = BuildTransformChains(root.transform, allWorldPts, strandRanges);

        if (_hideOriginal)
        {
            Undo.RecordObject(_source, "Hide Source");
            _source.SetActive(false);
        }

        // ── 5. DynamicBone auto-setup ─────────────────────────────────────
        if (applyDB && strandRoots != null)
        {
            if (DynamicBoneAvailable)
            {
                var colliders = _autoColliders ? PlaceColliders() : new List<Component>();
                foreach (var sr in strandRoots)
                    AttachDynamicBone(sr, colliders);
                Debug.Log($"[HairCards] DynamicBone on {strandRoots.Count} strands, " +
                          $"{colliders.Count} colliders.");
            }
            else
            {
                Debug.LogWarning("[HairCards] DynamicBone not found — physics skipped. " +
                                 "Import it from the Unity Asset Store.");
            }
        }

        Undo.CollapseUndoOperations(undoGroup);
        Selection.activeGameObject = root;
        EditorGUIUtility.PingObject(root);

        Debug.Log($"[HairCards] Done — {strandRanges.Count} strands, " +
                  $"{allWorldPts.Count} pts → '{root.name}'.");
    }

    // ══════════════════════════════════════════════════════════════════════
    //  Output: edge-only guide mesh (for HDRP Hair / re-export)
    // ══════════════════════════════════════════════════════════════════════

    void BuildGuideMesh(Transform parent, List<Vector3> worldPts,
                        List<(int a, int b)> edges, List<int> origIdx,
                        SkinnedMeshRenderer srcSmr)
    {
        var localPts = worldPts.Select(p => parent.InverseTransformPoint(p)).ToArray();
        var edgeArr  = new int[edges.Count * 2];
        for (int i = 0; i < edges.Count; i++)
        { edgeArr[i * 2] = edges[i].a; edgeArr[i * 2 + 1] = edges[i].b; }

        var guideMesh = new Mesh { name = parent.name };
        guideMesh.vertices = localPts;
        guideMesh.SetIndices(edgeArr, MeshTopology.Lines, 0);
        guideMesh.RecalculateBounds();

        string assetPath = AssetDatabase.GenerateUniqueAssetPath(
                               $"Assets/{parent.name}.asset");
        AssetDatabase.CreateAsset(guideMesh, assetPath);
        AssetDatabase.SaveAssets();

        var go = new GameObject(parent.name + "_Mesh");
        Undo.RegisterCreatedObjectUndo(go, "Guide Mesh GO");
        go.transform.SetParent(parent, worldPositionStays: false);

        if (_xferWeights && srcSmr != null)
        {
            BoneWeight[] srcBW = srcSmr.sharedMesh.boneWeights;
            if (srcBW != null && srcBW.Length > 0)
            {
                var newBW = new BoneWeight[localPts.Length];
                for (int i = 0; i < localPts.Length; i++)
                {
                    int oi = i < origIdx.Count ? origIdx[i] : 0;
                    if (oi < srcBW.Length) newBW[i] = srcBW[oi];
                }
                guideMesh.boneWeights = newBW;
                guideMesh.bindposes   = srcSmr.sharedMesh.bindposes;
                AssetDatabase.SaveAssets();

                var guideSMR        = go.AddComponent<SkinnedMeshRenderer>();
                guideSMR.sharedMesh = guideMesh;
                guideSMR.bones      = srcSmr.bones;
                guideSMR.rootBone   = srcSmr.rootBone;
                return;
            }
        }

        go.AddComponent<MeshFilter>().sharedMesh = guideMesh;
        var mat = new Material(Shader.Find("Hidden/Internal-Colored"))
                  { color = new Color(0.2f, 1f, 0.4f) };
        go.AddComponent<MeshRenderer>().sharedMaterial = mat;
    }

    // ══════════════════════════════════════════════════════════════════════
    //  Output: transform-chain hierarchy
    //
    //  Strand_000
    //    bone_00      ← root (highest Y = scalp attachment)
    //    bone_01
    //    …
    //    bone_NN
    //      bone_end   ← extrapolated tip; DynamicBone uses this as end node
    // ══════════════════════════════════════════════════════════════════════

    List<GameObject> BuildTransformChains(Transform parent,
                                          List<Vector3> worldPts,
                                          List<(int start, int count)> ranges)
    {
        var strandRoots = new List<GameObject>(ranges.Count);

        for (int si = 0; si < ranges.Count; si++)
        {
            var (strandStart, count) = ranges[si];

            // Strand_NNN — DynamicBone component lives here
            var strandRoot = new GameObject($"Strand_{si:000}");
            Undo.RegisterCreatedObjectUndo(strandRoot, "Strand");
            strandRoot.transform.SetParent(parent, worldPositionStays: false);
            strandRoots.Add(strandRoot);

            Transform prev = strandRoot.transform;
            for (int bi = 0; bi < count; bi++)
            {
                var bone = new GameObject($"bone_{bi:00}");
                Undo.RegisterCreatedObjectUndo(bone, "Bone");
                bone.transform.position = worldPts[strandStart + bi];
                bone.transform.SetParent(prev, worldPositionStays: true);
                prev = bone.transform;
            }

            // End bone — one extra segment past the tip for DynamicBone
            if (_addEndBone && count >= 2)
            {
                Vector3 tipPrev = worldPts[strandStart + count - 2];
                Vector3 tip     = worldPts[strandStart + count - 1];
                var endBone     = new GameObject("bone_end");
                Undo.RegisterCreatedObjectUndo(endBone, "End Bone");
                endBone.transform.position = tip + (tip - tipPrev);
                endBone.transform.SetParent(prev, worldPositionStays: true);
            }
        }

        return strandRoots;
    }

    // ══════════════════════════════════════════════════════════════════════
    //  Physics: auto-place DynamicBoneCollider on armature bones
    //
    //  Walks up from the hair mesh to find the first Animator ancestor,
    //  then searches all child Transforms for keyword matches.
    // ══════════════════════════════════════════════════════════════════════

    List<Component> PlaceColliders()
    {
        var placed = new List<Component>();
        if (!DynamicBoneAvailable || s_DBColliderType == null) return placed;

        Transform armRoot = FindArmatureRoot(_source.transform);
        if (armRoot == null)
        {
            Debug.LogWarning("[HairCards] No armature found — colliders skipped.");
            return placed;
        }

        string[] keywords = _colliderKeywords
                            .Split(',')
                            .Select(k => k.Trim().ToLowerInvariant())
                            .Where(k => k.Length > 0)
                            .ToArray();

        foreach (Transform t in armRoot.GetComponentsInChildren<Transform>(true))
        {
            if (!keywords.Any(kw => t.name.ToLowerInvariant().Contains(kw))) continue;
            if (t.GetComponent(s_DBColliderType) != null) continue;   // already has one

            var col = Undo.AddComponent(t.gameObject, s_DBColliderType) as Component;
            SetField(col, "m_Radius", _colliderRadius);
            SetField(col, "m_Center", Vector3.zero);

            // m_Bound = Outside (enum value 0)
            var boundField = s_DBColliderType.GetField("m_Bound");
            if (boundField != null)
                boundField.SetValue(col, Enum.ToObject(boundField.FieldType, 0));

            placed.Add(col);
        }

        return placed;
    }

    static Transform FindArmatureRoot(Transform t)
    {
        Transform cur = t.parent;
        while (cur != null)
        {
            if (cur.GetComponent<Animator>() != null) return cur;
            cur = cur.parent;
        }
        return t.parent;   // fallback: immediate parent
    }

    // ══════════════════════════════════════════════════════════════════════
    //  Physics: add and configure DynamicBone on a Strand_NNN GO
    //
    //  DynamicBone component fields set via reflection — no compile
    //  dependency on the third-party DynamicBone assembly.
    // ══════════════════════════════════════════════════════════════════════

    void AttachDynamicBone(GameObject strandRoot, List<Component> colliders)
    {
        var db = Undo.AddComponent(strandRoot, DBType) as Component;

        // m_Root = bone_00 (first child of Strand_NNN)
        if (strandRoot.transform.childCount > 0)
            SetField(db, "m_Root", strandRoot.transform.GetChild(0));

        SetField(db, "m_UpdateRate", 60f);
        SetField(db, "m_Damping",    _damping);
        SetField(db, "m_Elasticity", _elasticity);
        SetField(db, "m_Stiffness",  _stiffness);
        SetField(db, "m_Inert",      _inert);
        SetField(db, "m_Radius",     _radius);
        SetField(db, "m_Gravity",    _gravity);

        // m_FreezeAxis = None (0)
        var freezeField = DBType.GetField("m_FreezeAxis");
        if (freezeField != null)
            freezeField.SetValue(db, Enum.ToObject(freezeField.FieldType, 0));

        // m_Colliders = List<DynamicBoneColliderBase> with all placed colliders
        if (colliders.Count > 0 && s_DBColliderBaseType != null)
        {
            var listType = typeof(List<>).MakeGenericType(s_DBColliderBaseType);
            var list     = Activator.CreateInstance(listType);
            var addMeth  = listType.GetMethod("Add");
            foreach (var c in colliders)
                if (c != null) addMeth?.Invoke(list, new object[] { c });
            SetField(db, "m_Colliders", list);
        }
    }

    // ══════════════════════════════════════════════════════════════════════
    //  Algorithm: double-BFS longest path
    //  Direct port of find_longest_path + bfs (hair_cards_to_curve.py)
    // ══════════════════════════════════════════════════════════════════════

    (Dictionary<int, int> dist, Dictionary<int, int> prev)
        BFS(Dictionary<int, HashSet<int>> adj, int seed, HashSet<int> subset)
    {
        var dist = new Dictionary<int, int>(subset.Count);
        var prev = new Dictionary<int, int>(subset.Count);
        dist[seed] = 0;
        var q = new Queue<int>();
        q.Enqueue(seed);
        while (q.Count > 0)
        {
            int v = q.Dequeue();
            foreach (int n in adj[v])
            {
                if (subset.Contains(n) && !dist.ContainsKey(n))
                {
                    dist[n] = dist[v] + 1;
                    prev[n] = v;
                    q.Enqueue(n);
                }
            }
        }
        return (dist, prev);
    }

    List<int> FindLongestPath(Dictionary<int, HashSet<int>> adj, List<int> comp)
    {
        var subset     = new HashSet<int>(comp);
        var (d0, _)    = BFS(adj, comp[0], subset);
        int far1       = comp.OrderByDescending(
                             v => d0.TryGetValue(v, out int d) ? d : -1).First();
        var (d1, prev) = BFS(adj, far1, subset);
        int far2       = comp.OrderByDescending(
                             v => d1.TryGetValue(v, out int d) ? d : -1).First();

        var path = new List<int>();
        int cur  = far2;
        while (true)
        {
            path.Add(cur);
            if (!prev.TryGetValue(cur, out int p)) break;
            cur = p;
        }
        path.Reverse();
        return path;
    }

    // ══════════════════════════════════════════════════════════════════════
    //  Algorithm: Laplacian smooth  — matches Python smooth_path()
    // ══════════════════════════════════════════════════════════════════════

    List<Vector3> SmoothPath(List<Vector3> pts)
    {
        for (int it = 0; it < _smoothIter; it++)
        {
            var s = new List<Vector3>(pts.Count) { pts[0] };
            for (int i = 1; i < pts.Count - 1; i++)
                s.Add(Vector3.Lerp(pts[i], (pts[i - 1] + pts[i + 1]) * 0.5f, _smoothStr));
            s.Add(pts[pts.Count - 1]);
            pts = s;
        }
        return pts;
    }

    // Reverse in-place: highest-Y first  — matches ensure_root_at_top()
    static void EnsureRootAtTop(List<Vector3> pts)
    {
        if (pts.Count >= 2 && pts[0].y < pts[pts.Count - 1].y)
            pts.Reverse();
    }

    // ══════════════════════════════════════════════════════════════════════
    //  Reflection helper
    // ══════════════════════════════════════════════════════════════════════

    static void SetField(object obj, string name, object value)
    {
        FieldInfo fi = obj?.GetType().GetField(name);
        if (fi == null) return;
        // Coerce double → float (DynamicBone uses float throughout)
        if (fi.FieldType == typeof(float) && value is double d) value = (float)d;
        fi.SetValue(obj, value);
    }
}
