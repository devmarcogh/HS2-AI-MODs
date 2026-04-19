using System;
using System.Collections.Generic;
using Studio;
using UnityEngine;

namespace StudioModsMSG
{
    class ClothPhysicsPanel : IModulePanelUI
    {
        private readonly ClothPhysicsModule logic = new ClothPhysicsModule();

        private sealed class ClothConfigSnapshot
        {
            public float StretchStiffness;
            public float BendStiffness;
            public float Damping;
            public float Thickness;
            public float RestInflate;
            public float Gravity;
            public float Weight;
            public float Compression;
            public float Elasticity;
            public float Substeps;
            public float Iterations;
            public bool ClothToCloth;
            public List<string> PinSourceBoneNames;
            public bool ShowPinBoneGizmos;
        }

        private sealed class BoneNode
        {
            public string Name;
            public readonly List<BoneNode> Children = new List<BoneNode>();
        }

        private enum NodeSelectionState
        {
            None,
            Partial,
            All
        }

        // GUI state
        private SelectionContext lastSelection;
        private ClothMeshState   selectedMesh;
        private string           selectedCategoryId;
        private string           selectedMeshName;
        private Vector2          listScroll = Vector2.zero;
        private Vector2          boneScroll = Vector2.zero;
        private const float      ListHeight = 170f;
        private readonly Dictionary<ClothMeshState, HashSet<string>> expandedBoneNodesByMesh =
            new Dictionary<ClothMeshState, HashSet<string>>();
        private readonly HashSet<string> previouslyActiveMeshKeys = new HashSet<string>(StringComparer.OrdinalIgnoreCase);

        // Body colliders section state
        private Dictionary<string, bool> _sdfGroupEnabled;

        // Scene colliders section state
        private bool _sceneCollidersExpanded = false;
        private ClothColliderProxy _selectedProxy;

        public string ModuleId => FeatureModuleIds.ClothPhysics;

        public bool GetDefaultEnabledState() => true;
        public void SyncEnabledState(ref bool enabled) { }
        public void OnToggleChanged(bool enabled)
        {
            if (lastSelection == null) return;

            if (!enabled)
            {
                RememberActiveMeshes(lastSelection);
                logic.DisableAllSimulation(lastSelection, true);
                selectedMesh = null;
                return;
            }

            RestorePreviouslyActiveMeshes(lastSelection);
        }

        private static string MakeMeshKey(ClothMeshState mesh)
        {
            if (mesh == null) return string.Empty;
            return (mesh.CategoryId ?? string.Empty) + "|" + (mesh.MeshName ?? string.Empty);
        }

        private string MakeSelectedMeshKey()
        {
            return (selectedCategoryId ?? string.Empty) + "|" + (selectedMeshName ?? string.Empty);
        }

        private ClothMeshState FindMeshByKey(IReadOnlyList<ClothPhysicsEntry> entries, string meshKey)
        {
            if (entries == null || string.IsNullOrEmpty(meshKey)) return null;

            for (int ei = 0; ei < entries.Count; ei++)
            {
                var meshes = entries[ei].Meshes;
                for (int mi = 0; mi < meshes.Count; mi++)
                {
                    ClothMeshState mesh = meshes[mi];
                    if (string.Equals(MakeMeshKey(mesh), meshKey, StringComparison.OrdinalIgnoreCase))
                        return mesh;
                }
            }

            return null;
        }

        private ClothMeshState ResolveSelectedMesh(SelectionContext selection)
        {
            IReadOnlyList<ClothPhysicsEntry> entries = logic.GetEntries(selection);
            if (entries == null || entries.Count == 0) return null;

            string key = MakeSelectedMeshKey();
            if (!string.IsNullOrEmpty(key) && key != "|")
            {
                ClothMeshState resolved = FindMeshByKey(entries, key);
                if (resolved != null)
                {
                    selectedMesh = resolved;
                    return resolved;
                }
            }

            if (selectedMesh != null)
            {
                ClothMeshState resolved = FindMeshByKey(entries, MakeMeshKey(selectedMesh));
                if (resolved != null)
                {
                    selectedMesh = resolved;
                    selectedCategoryId = resolved.CategoryId;
                    selectedMeshName   = resolved.MeshName;
                    return resolved;
                }
            }

            return null;
        }

        private void RememberSelectedMesh(ClothMeshState mesh)
        {
            selectedMesh = mesh;
            selectedCategoryId = mesh != null ? mesh.CategoryId : null;
            selectedMeshName   = mesh != null ? mesh.MeshName : null;
        }

        private void RememberActiveMeshes(SelectionContext selection)
        {
            previouslyActiveMeshKeys.Clear();

            IReadOnlyList<ClothPhysicsEntry> entries = logic.GetEntries(selection);
            if (entries == null) return;

            for (int ei = 0; ei < entries.Count; ei++)
            {
                var meshes = entries[ei].Meshes;
                for (int mi = 0; mi < meshes.Count; mi++)
                {
                    ClothMeshState mesh = meshes[mi];
                    if (mesh != null && mesh.IsActive)
                        previouslyActiveMeshKeys.Add(MakeMeshKey(mesh));
                }
            }
        }

        private void RestorePreviouslyActiveMeshes(SelectionContext selection)
        {
            if (previouslyActiveMeshKeys.Count == 0) return;

            IReadOnlyList<ClothPhysicsEntry> entries = logic.GetEntries(selection);
            if (entries == null) return;

            for (int ei = 0; ei < entries.Count; ei++)
            {
                var meshes = entries[ei].Meshes;
                for (int mi = 0; mi < meshes.Count; mi++)
                {
                    ClothMeshState mesh = meshes[mi];
                    if (mesh != null && previouslyActiveMeshKeys.Contains(MakeMeshKey(mesh)) && !mesh.IsActive)
                        logic.ActivateMesh(selection, mesh);
                }
            }

            if (!string.IsNullOrEmpty(selectedCategoryId) || !string.IsNullOrEmpty(selectedMeshName))
                selectedMesh = FindMeshByKey(entries, MakeSelectedMeshKey());
        }

        public bool TryCopyConfig(SelectionContext selection, out object config)
        {
            config = null;
            ClothMeshState mesh = ResolveSelectedMesh(selection);
            if (mesh == null || mesh.Params == null) return false;

            ClothPhysicsParams p = mesh.Params;
            config = new ClothConfigSnapshot
            {
                StretchStiffness = p.StretchStiffness,
                BendStiffness = p.BendStiffness,
                Damping = p.Damping,
                Thickness = p.Thickness,
                RestInflate = p.RestInflate,
                Gravity = p.Gravity,
                Weight = p.Weight,
                Compression = p.Compression,
                Elasticity = p.Elasticity,
                Substeps = p.Substeps,
                Iterations = p.Iterations,
                ClothToCloth = p.ClothToCloth,
                PinSourceBoneNames = new List<string>(mesh.PinSourceBoneNames ?? new List<string>()),
                ShowPinBoneGizmos = mesh.ShowPinBoneGizmos,
            };
            return true;
        }

        public bool TryPasteConfig(SelectionContext selection, object config)
        {
            ClothConfigSnapshot snap = config as ClothConfigSnapshot;
            ClothMeshState mesh = ResolveSelectedMesh(selection);
            if (snap == null || mesh == null || mesh.Params == null) return false;

            ClothPhysicsParams p = mesh.Params;
            p.StretchStiffness = snap.StretchStiffness;
            p.BendStiffness = snap.BendStiffness;
            p.Damping = snap.Damping;
            p.Thickness = snap.Thickness;
            p.RestInflate = snap.RestInflate;
            p.Gravity = snap.Gravity;
            p.Weight = snap.Weight;
            p.Compression = snap.Compression;
            p.Elasticity = snap.Elasticity;
            p.Substeps = snap.Substeps;
            p.Iterations = snap.Iterations;
            p.ClothToCloth = snap.ClothToCloth;
            mesh.ShowPinBoneGizmos = snap.ShowPinBoneGizmos;

            if (mesh.PinSourceBoneNames == null)
                mesh.PinSourceBoneNames = new List<string>();
            else
                mesh.PinSourceBoneNames.Clear();

            if (snap.PinSourceBoneNames != null)
            {
                for (int i = 0; i < snap.PinSourceBoneNames.Count; i++)
                    AddBone(mesh.PinSourceBoneNames, snap.PinSourceBoneNames[i]);
            }

            RememberSelectedMesh(mesh);
            logic.RecomputePins(selection, mesh);
            return true;
        }

        // Human-readable names for clothing slots
        private static string FriendlySlotName(string catId)
        {
            switch (catId)
            {
                case "Top":     return "Top";
                case "Bot":     return "Bottom";
                case "Inner_t": return "Inner Top";
                case "Inner_b": return "Inner Bottom";
                case "Gloves":  return "Gloves";
                case "Panst":   return "Pantyhose";
                case "Socks":   return "Socks";
                case "Shoes":   return "Shoes";
                default:        return catId ?? "?";
            }
        }

        public void Draw(SelectionContext selection, BaseUI host)
        {
            if (!logic.CanHandle(selection))
            {
                GUILayout.Label("Cloth Physics applies to characters with clothing.", host.HintStyleRef);
                return;
            }

            // Detect selection change → clear selected mesh
            if (selection != lastSelection)
            {
                lastSelection = selection;
                RememberSelectedMesh(null);
                previouslyActiveMeshKeys.Clear();
            }

            IReadOnlyList<ClothPhysicsEntry> entries = logic.GetEntries(selection);
            selectedMesh = ResolveSelectedMesh(selection);

            if (entries == null || entries.Count == 0)
            {
                GUILayout.Label("No cloth items found on this character. Make sure clothing is equipped.", host.HintStyleRef);
                GUILayout.Space(6f);
                if (GUILayout.Button("Refresh", GUILayout.Height(26f)))
                    logic.RefreshEntries(selection);
                return;
            }

            // ── Body Colliders ────────────────────────────────────────────
            DrawAutoCollidersSection(selection, host);

            host.DrawDivider();

            // ── Cloth Meshes ──────────────────────────────────────────────
            GUILayout.Label("CLOTH MESHES", host.HintStyleRef);
            GUILayout.Space(2f);

            listScroll = GUILayout.BeginScrollView(listScroll, GUILayout.Height(ListHeight));

            foreach (ClothPhysicsEntry entry in entries)
            {
                string friendlyName = FriendlySlotName(entry.CategoryId);
                string arrow  = entry.IsExpanded ? "▾" : "▸";
                if (GUILayout.Button(arrow + "  " + friendlyName, host.HintStyleRef, GUILayout.Height(22f)))
                    entry.IsExpanded = !entry.IsExpanded;

                if (!entry.IsExpanded) continue;

                GUILayout.Space(1f);
                foreach (ClothMeshState ms in entry.Meshes)
                {
                    bool isSelected = ms == selectedMesh;
                    GUIStyle rowStyle = isSelected ? host.ActiveOptionStyleRef : host.HintStyleRef;

                    GUILayout.BeginHorizontal();
                    GUILayout.Space(14f);

                    // Sim on/off toggle
                    bool newActive = GUILayout.Toggle(ms.IsActive, string.Empty, GUILayout.Width(16f));
                    if (newActive != ms.IsActive)
                        logic.ToggleMesh(selection, ms);

                    // Per-mesh pause icon (only when active)
                    if (ms.IsActive)
                    {
                        string pauseIcon = ms.SimulationPaused ? "▶" : "❚❚";
                        if (GUILayout.Button(pauseIcon, host.HintStyleRef,
                                GUILayout.Width(20f), GUILayout.Height(18f)))
                            ms.SimulationPaused = !ms.SimulationPaused;
                    }
                    else
                    {
                        GUILayout.Space(22f);
                    }

                    // Mesh name — highlighted when selected
                    string stateTag = !ms.IsActive       ? " [off]"
                                    : ms.SimulationPaused ? " [paused]"
                                    :                       "";
                    string label = ms.MeshName + stateTag;
                    if (GUILayout.Button(label, rowStyle, GUILayout.ExpandWidth(true), GUILayout.Height(20f)))
                        RememberSelectedMesh(selectedMesh == ms ? null : ms);

                    GUILayout.EndHorizontal();
                    GUILayout.Space(1f);
                }
                GUILayout.Space(3f);
            }

            GUILayout.EndScrollView();

            GUILayout.Space(2f);
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Refresh", GUILayout.Height(22f), GUILayout.Width(70f)))
            {
                RememberSelectedMesh(null);
                logic.RefreshEntries(selection);
            }
            GUILayout.Label("Toggle sim  ·  ❚❚/▶ per mesh  ·  click to edit", host.HintStyleRef);
            GUILayout.EndHorizontal();

            // ── Per-mesh settings ─────────────────────────────────────────
            if (selectedMesh == null) return;

            host.DrawDivider();

            // Compact selected-mesh status bar
            string selectedFriendly = FriendlySlotName(selectedMesh.CategoryId) + " / " + selectedMesh.MeshName;
            string statusText = !selectedMesh.IsActive       ? "Inactive"
                              : selectedMesh.SimulationPaused ? "Paused"
                              :                                  "Simulating";
            Color statusCol  = !selectedMesh.IsActive || selectedMesh.SimulationPaused
                               ? host.WarningColorRef : host.SuccessColorRef;

            GUILayout.BeginHorizontal();
            GUILayout.Label(selectedFriendly, host.HintStyleRef, GUILayout.ExpandWidth(true));
            GUILayout.Label(statusText, new GUIStyle(host.HintStyleRef) { normal = { textColor = statusCol } }, GUILayout.Width(80f));
            if (selectedMesh.IsActive)
            {
                string btnLabel = selectedMesh.SimulationPaused ? "▶" : "❚❚";
                if (GUILayout.Button(btnLabel, host.HintStyleRef, GUILayout.Width(28f), GUILayout.Height(20f)))
                    selectedMesh.SimulationPaused = !selectedMesh.SimulationPaused;
            }
            GUILayout.EndHorizontal();

            GUILayout.Space(6f);

            // ── Simulation Mode ───────────────────────────────────────────
            GUILayout.Label("SIMULATION MODE", host.HintStyleRef);
            GUILayout.Space(2f);
            GUILayout.BeginHorizontal();
            {
                bool isCont   = selectedMesh.SimulationMode == ClothSimulationMode.Continuous;
                bool isManual = selectedMesh.SimulationMode == ClothSimulationMode.ManualDeformation;
                GUIStyle onStyle  = host.ActiveOptionStyleRef;
                GUIStyle offStyle = host.HintStyleRef;

                if (GUILayout.Button("Continuous", isCont ? onStyle : offStyle,
                        GUILayout.Height(26f), GUILayout.ExpandWidth(true)))
                    selectedMesh.SimulationMode = ClothSimulationMode.Continuous;

                GUILayout.Space(2f);

                if (GUILayout.Button("Manual Deformation", isManual ? onStyle : offStyle,
                        GUILayout.Height(26f), GUILayout.ExpandWidth(true)))
                    selectedMesh.SimulationMode = ClothSimulationMode.ManualDeformation;
            }
            GUILayout.EndHorizontal();

            if (selectedMesh.SimulationMode == ClothSimulationMode.ManualDeformation)
            {
                GUILayout.Space(3f);
                DrawFloatRow(host, "Deform Radius", ref selectedMesh.DeformRadius, 0.02f, 0.8f);

                // Live awake-vertex count
                int awake = 0;
                if (selectedMesh.TriggerCooldown != null)
                    for (int j = 0; j < selectedMesh.TriggerCooldown.Length; j++)
                        if (selectedMesh.TriggerCooldown[j] > 0) awake++;
                string awakeText = selectedMesh.IsActive
                    ? (selectedMesh.IsDragging ? "Dragging — " : "") + awake + " / " + selectedMesh.VertCount + " verts awake"
                    : "Inactive";
                host.DrawStatRow("Status", awakeText,
                    awake > 0 ? host.SuccessColorRef : host.WarningColorRef);

                string keyText = StudioCharaEditor.KeyManualDeform != null
                    ? StudioCharaEditor.KeyManualDeform.Value.ToString()
                    : "Shift+D";
                GUILayout.Space(2f);
                GUILayout.Label(
                    "Hold  " + keyText + "  +  Right-Mouse  near the mesh to sculpt.\n" +
                    "Vertices within Deform Radius are attracted toward the cursor.\n" +
                    "Release to pause.",
                    host.HintStyleRef);
                GUILayout.Space(4f);
            }

            host.DrawDivider();

            // ── Physics parameters ────────────────────────────────────────
            ClothPhysicsParams p = selectedMesh.Params;

            DrawFloatRow(host, "Stretch",     ref p.StretchStiffness, 0f,     100f);
            DrawFloatRow(host, "Bending",     ref p.BendStiffness,    0f,       50f);
            DrawFloatRow(host, "Damping",     ref p.Damping,          0.5f,     20f);
            DrawFloatRow(host, "Thickness",   ref p.Thickness,        0.001f,   0.4f);
            DrawFloatRow(host, "Rest Inflate", ref p.RestInflate,     0f,       0.04f);
            DrawFloatRow(host, "Gravity",     ref p.Gravity,         -30f,      0f);
            DrawFloatRow(host, "Weight",      ref p.Weight,           0.1f,     3f);
            DrawFloatRow(host, "Compression", ref p.Compression,      0f,       1f);

            host.DrawDivider();

            // ── Substeps / Iterations ─────────────────────────────────────
            GUILayout.Label("QUALITY", host.HintStyleRef);
            GUILayout.Space(2f);
            DrawPresetRow(host, "Substeps",   ref p.Substeps);
            DrawPresetRow(host, "Iterations", ref p.Iterations);

            host.DrawDivider();

            // ── Cloth-to-cloth ────────────────────────────────────────────
            GUILayout.BeginHorizontal();
            GUILayout.Label("CLOTH-TO-CLOTH", host.HintStyleRef, GUILayout.ExpandWidth(true));
            if (DrawModeButton(host, p.ClothToCloth ? "ON" : "OFF", p.ClothToCloth, 48f))
                p.ClothToCloth = !p.ClothToCloth;
            GUILayout.EndHorizontal();

            host.DrawDivider();

            // ── Pin Source Bones ──────────────────────────────────────────
            GUILayout.Label("PIN SOURCE BONES", host.HintStyleRef);
            GUILayout.Space(2f);

            DrawBoneTree(host, selectedMesh);

            GUILayout.Space(3f);
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("All",  GUILayout.Height(22f), GUILayout.Width(42f)))
                SelectAllBones(selectedMesh);
            if (GUILayout.Button("None", GUILayout.Height(22f), GUILayout.Width(42f)))
                selectedMesh.PinSourceBoneNames.Clear();
            GUILayout.Space(4f);
            if (GUILayout.Button("Apply Pins", GUILayout.Height(22f), GUILayout.Width(88f)))
                logic.RecomputePins(lastSelection, selectedMesh);
            GUILayout.Space(4f);
            selectedMesh.ShowPinBoneGizmos = GUILayout.Toggle(
                selectedMesh.ShowPinBoneGizmos, "Gizmos", GUILayout.Width(56f));
            GUILayout.EndHorizontal();

            int pinned = 0;
            if (selectedMesh.IsPinned != null)
                for (int i = 0; i < selectedMesh.IsPinned.Length; i++)
                    if (selectedMesh.IsPinned[i]) pinned++;

            host.DrawStatRow("Selected Bones", selectedMesh.PinSourceBoneNames.Count.ToString(), host.HintStyleRef.normal.textColor);
            host.DrawStatRow("Pinned Verts",   pinned.ToString(),                                 host.HintStyleRef.normal.textColor);
        }


        private void DrawBoneTree(BaseUI host, ClothMeshState ms)
        {
            List<BoneNode> roots = BuildBoneTree(ms);

            boneScroll = GUILayout.BeginScrollView(boneScroll, GUILayout.Height(180f));
            for (int i = 0; i < roots.Count; i++)
                DrawBoneNodeRow(host, ms, roots[i], 0);
            GUILayout.EndScrollView();
        }

        private void DrawBoneNodeRow(BaseUI host, ClothMeshState ms, BoneNode node, int depth)
        {
            HashSet<string> expanded = GetExpandedSet(ms);
            bool hasChildren = node.Children.Count > 0;
            bool isExpanded = expanded.Contains(node.Name);
            NodeSelectionState state = GetNodeSelectionState(ms.PinSourceBoneNames, node);

            GUILayout.BeginHorizontal();
            GUILayout.Space(8f + depth * 14f);

            if (hasChildren)
            {
                string fold = isExpanded ? "▼" : "▶";
                if (GUILayout.Button(fold, GUILayout.Width(20f), GUILayout.Height(20f)))
                {
                    if (isExpanded) expanded.Remove(node.Name);
                    else expanded.Add(node.Name);
                }
            }
            else
            {
                GUILayout.Space(22f);
            }

            string stateText = state == NodeSelectionState.All ? "[x]" : (state == NodeSelectionState.Partial ? "[-]" : "[ ]");
            string rowText = stateText + " " + node.Name;
            if (GUILayout.Button(rowText, host.HintStyleRef, GUILayout.ExpandWidth(true), GUILayout.Height(20f)))
            {
                if (state == NodeSelectionState.All) RemoveNodeRecursive(ms.PinSourceBoneNames, node);
                else AddNodeRecursive(ms.PinSourceBoneNames, node);
            }

            GUILayout.EndHorizontal();

            if (hasChildren && isExpanded)
            {
                for (int i = 0; i < node.Children.Count; i++)
                    DrawBoneNodeRow(host, ms, node.Children[i], depth + 1);
            }
        }

        private void SelectAllBones(ClothMeshState ms)
        {
            ms.PinSourceBoneNames.Clear();
            List<BoneNode> roots = BuildBoneTree(ms);
            for (int i = 0; i < roots.Count; i++)
                AddNodeRecursive(ms.PinSourceBoneNames, roots[i]);
        }

        private List<BoneNode> BuildBoneTree(ClothMeshState ms)
        {
            var roots = new List<BoneNode>();
            if (ms == null || ms.SkinBones == null) return roots;

            var validNames = GetValidBoneNames(ms);
            var boneByName = new Dictionary<string, Transform>(StringComparer.OrdinalIgnoreCase);
            for (int i = 0; i < ms.SkinBones.Length; i++)
            {
                Transform b = ms.SkinBones[i];
                if (b == null || string.IsNullOrEmpty(b.name)) continue;
                if (!validNames.Contains(b.name)) continue;
                if (!boneByName.ContainsKey(b.name)) boneByName[b.name] = b;
            }

            var nodeByName = new Dictionary<string, BoneNode>(StringComparer.OrdinalIgnoreCase);
            foreach (var kv in boneByName)
                nodeByName[kv.Key] = new BoneNode { Name = kv.Key };

            foreach (var kv in boneByName)
            {
                string childName = kv.Key;
                Transform parent = kv.Value.parent;
                BoneNode childNode = nodeByName[childName];

                // compress hierarchy to the nearest valid ancestor so we keep
                // tree behavior without introducing bones that have no vertices.
                BoneNode parentNode = null;
                while (parent != null)
                {
                    if (!string.IsNullOrEmpty(parent.name) && nodeByName.TryGetValue(parent.name, out parentNode))
                        break;
                    parent = parent.parent;
                }

                if (parentNode != null)
                    parentNode.Children.Add(childNode);
                else
                    roots.Add(childNode);
            }

            SortTreeRecursive(roots);
            return roots;
        }

        private static HashSet<string> GetValidBoneNames(ClothMeshState ms)
        {
            var names = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            if (ms == null || ms.SkinBones == null) return names;

            bool hasDomSet = ms.BonesWithDominantVertices != null && ms.BonesWithDominantVertices.Count > 0;
            for (int i = 0; i < ms.SkinBones.Length; i++)
            {
                Transform bone = ms.SkinBones[i];
                if (bone == null || string.IsNullOrEmpty(bone.name)) continue;
                if (hasDomSet && !ms.BonesWithDominantVertices.Contains(bone.name)) continue;
                names.Add(bone.name);
            }
            return names;
        }

        private static void SortTreeRecursive(List<BoneNode> nodes)
        {
            nodes.Sort((a, b) => string.Compare(a.Name, b.Name, StringComparison.OrdinalIgnoreCase));
            for (int i = 0; i < nodes.Count; i++)
                SortTreeRecursive(nodes[i].Children);
        }

        private HashSet<string> GetExpandedSet(ClothMeshState ms)
        {
            HashSet<string> expanded;
            if (!expandedBoneNodesByMesh.TryGetValue(ms, out expanded))
            {
                expanded = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
                expandedBoneNodesByMesh[ms] = expanded;
            }
            return expanded;
        }

        private static void AddNodeRecursive(List<string> selected, BoneNode node)
        {
            AddBone(selected, node.Name);
            for (int i = 0; i < node.Children.Count; i++)
                AddNodeRecursive(selected, node.Children[i]);
        }

        private static void RemoveNodeRecursive(List<string> selected, BoneNode node)
        {
            RemoveBone(selected, node.Name);
            for (int i = 0; i < node.Children.Count; i++)
                RemoveNodeRecursive(selected, node.Children[i]);
        }

        private static NodeSelectionState GetNodeSelectionState(List<string> selected, BoneNode node)
        {
            int total = CountNodesRecursive(node);
            int selectedCount = CountSelectedRecursive(selected, node);
            if (selectedCount <= 0) return NodeSelectionState.None;
            if (selectedCount >= total) return NodeSelectionState.All;
            return NodeSelectionState.Partial;
        }

        private static int CountNodesRecursive(BoneNode node)
        {
            int count = 1;
            for (int i = 0; i < node.Children.Count; i++)
                count += CountNodesRecursive(node.Children[i]);
            return count;
        }

        private static int CountSelectedRecursive(List<string> selected, BoneNode node)
        {
            int count = ContainsBone(selected, node.Name) ? 1 : 0;
            for (int i = 0; i < node.Children.Count; i++)
                count += CountSelectedRecursive(selected, node.Children[i]);
            return count;
        }

        private static bool ContainsBone(List<string> bones, string boneName)
        {
            if (bones == null || string.IsNullOrEmpty(boneName)) return false;
            for (int i = 0; i < bones.Count; i++)
                if (string.Equals(bones[i], boneName, StringComparison.OrdinalIgnoreCase))
                    return true;
            return false;
        }

        private static void AddBone(List<string> bones, string boneName)
        {
            if (bones == null || string.IsNullOrEmpty(boneName)) return;
            if (!ContainsBone(bones, boneName)) bones.Add(boneName);
        }

        private static void RemoveBone(List<string> bones, string boneName)
        {
            if (bones == null || string.IsNullOrEmpty(boneName)) return;
            for (int i = bones.Count - 1; i >= 0; i--)
                if (string.Equals(bones[i], boneName, StringComparison.OrdinalIgnoreCase))
                    bones.RemoveAt(i);
        }

        private static void DrawFloatRow(BaseUI host, string label, ref float value, float min, float max)
        {
            Rect rowRect = GUILayoutUtility.GetRect(10f, 40f, GUILayout.ExpandWidth(true));
            GUI.Label(new Rect(rowRect.x + 4f, rowRect.y + 4f, 100f, 16f),  label,              host.HintStyleRef);
            GUI.Label(new Rect(rowRect.xMax - 70f, rowRect.y + 4f, 66f, 16f), value.ToString("0.###"), host.HintStyleRef);
            float newVal = GUI.HorizontalSlider(
                new Rect(rowRect.x + 4f, rowRect.y + 22f, rowRect.width - 80f, 14f),
                value, min, max);
            if (!Mathf.Approximately(newVal, value)) value = newVal;
        }

        private static void DrawPresetRow(BaseUI host, string label, ref float value)
        {
            float[] options = { 1f, 2f, 4f, 6f, 8f };
            GUILayout.BeginHorizontal();
            GUILayout.Label(label, host.HintStyleRef, GUILayout.Width(80f));
            for (int i = 0; i < options.Length; i++)
            {
                float v = options[i];
                bool selected = Mathf.Abs(value - v) <= 0.0001f;
                string txt = v.ToString("0");
                if (GUILayout.Button(txt,
                        selected ? host.ActiveOptionStyleRef : host.HintStyleRef,
                        GUILayout.Width(28f), GUILayout.Height(22f)))
                    value = v;
            }
            GUILayout.EndHorizontal();
        }

        // ── Scene Colliders section ────────────────────────────────────────
        // (Compact quick-view; full editing available by selecting a collider in the
        //  workspace tree — the dedicated Cloth Collider panel will open automatically.)

        private void DrawSceneCollidersSection(BaseUI host)
        {
            string arrow = _sceneCollidersExpanded ? "▼" : "▶";
            if (GUILayout.Button(arrow + "  Scene Colliders", host.HintStyleRef, GUILayout.Height(24f)))
                _sceneCollidersExpanded = !_sceneCollidersExpanded;

            if (!_sceneCollidersExpanded) return;

            GUILayout.Space(4f);

            var proxies = ClothColliderProxy.All;
            host.DrawStatRow("Active colliders", proxies.Count.ToString(), host.SuccessColorRef);
            GUILayout.Space(4f);

            GUILayout.BeginHorizontal();
            if (GUILayout.Button("+ Add Capsule Collider", GUILayout.Height(26f), GUILayout.Width(180f)))
            {
                Vector3 spawnPos = Vector3.zero;
                if (selectedMesh != null && selectedMesh.Renderer != null)
                    spawnPos = selectedMesh.Renderer.bounds.center;
                _selectedProxy = ClothColliderProxy.CreateInWorkspace(spawnPos);
            }
            GUILayout.EndHorizontal();

            GUILayout.Space(4f);

            // Compact list with quick remove
            ClothColliderProxy toRemove = null;
            for (int i = 0; i < proxies.Count; i++)
            {
                var proxy = proxies[i];
                if (proxy == null) continue;

                GUILayout.BeginHorizontal();
                GUILayout.Space(8f);
                string name  = proxy.ColliderName;
                string mode  = proxy.MagneticMode == ColliderMagneticMode.Attract ? " [+]"
                             : proxy.MagneticMode == ColliderMagneticMode.Off     ? " [x]"
                             :                                                       "";
                GUILayout.Label(name + mode, host.HintStyleRef, GUILayout.ExpandWidth(true));
                if (GUILayout.Button("X", GUILayout.Width(24f), GUILayout.Height(22f)))
                    toRemove = proxy;
                GUILayout.EndHorizontal();
            }

            if (toRemove != null)
                toRemove.RemoveFromWorkspace();

            GUILayout.Space(2f);
            GUILayout.Label("Select a collider in the workspace tree to edit its properties.", host.HintStyleRef);
        }

        // ── Body Colliders section ─────────────────────────────────────────

        private Dictionary<string, bool> GetOrCreateSDFGroups()
        {
            if (_sdfGroupEnabled != null) return _sdfGroupEnabled;

            _sdfGroupEnabled = new Dictionary<string, bool>(StringComparer.OrdinalIgnoreCase);
            for (int i = 0; i < ClothSoftBodyRuntime.SDFBoneGroupNames.Length; i++)
            {
                string groupName = ClothSoftBodyRuntime.SDFBoneGroupNames[i];
                _sdfGroupEnabled[groupName] = string.Equals(groupName, "Breasts", StringComparison.OrdinalIgnoreCase);
            }
            return _sdfGroupEnabled;
        }

        private void DrawAutoCollidersSection(SelectionContext selection, BaseUI host)
        {
            // Section header  (always visible — no expand toggle, it's a flat top section)
            GUILayout.Label("BODY COLLIDERS", host.HintStyleRef);
            GUILayout.Space(3f);

            ClothSoftBodyRuntime rt = logic.GetOrCreateRuntime(selection);

            bool useMirror = rt != null && rt.UseMirrorColliders;
            bool useSDF    = rt != null && rt.UseSDFColliders;

            // ── Collider toggles: HS2PE Colliders  |  SDF ─────────────────
            GUILayout.BeginHorizontal();
            GUILayout.Label("Active:", host.HintStyleRef, GUILayout.Width(52f));
            if (DrawModeButton(host, "HS2PE Colliders", useMirror, 112f))
                if (rt != null) rt.UseMirrorColliders = !rt.UseMirrorColliders;
            GUILayout.Space(4f);
            if (DrawModeButton(host, "SDF", useSDF, 44f))
                if (rt != null) rt.UseSDFColliders = !rt.UseSDFColliders;
            GUILayout.EndHorizontal();
            GUILayout.Space(4f);

            // Per-bone-group selector (only shown when SDF is active)
            if (useSDF)
            {
                var groups = GetOrCreateSDFGroups();

                for (int gi = 0; gi < ClothSoftBodyRuntime.SDFBoneGroupNames.Length; gi++)
                {
                    string groupName   = ClothSoftBodyRuntime.SDFBoneGroupNames[gi];
                    bool   enabledGroup = groups[groupName];

                    GUILayout.BeginHorizontal();
                    GUILayout.Space(10f);
                    GUILayout.Label(groupName, host.HintStyleRef, GUILayout.Width(68f));

                    if (DrawModeButton(host, "Off", !enabledGroup, 40f)) groups[groupName] = false;
                    if (DrawModeButton(host, "SDF",  enabledGroup, 40f)) groups[groupName] = true;
                    GUILayout.EndHorizontal();
                }

                GUILayout.Space(2f);
                // SDF quality selector
                if (rt != null)
                {
                    GUILayout.BeginHorizontal();
                    GUILayout.Space(10f);
                    GUILayout.Label("Quality:", host.HintStyleRef, GUILayout.Width(56f));
                    if (DrawModeButton(host, "Low",   rt.SDFQuality == 1, 36f)) rt.SDFQuality = 1;
                    if (DrawModeButton(host, "Med",   rt.SDFQuality == 2, 36f)) rt.SDFQuality = 2;
                    if (DrawModeButton(host, "High",  rt.SDFQuality == 3, 38f)) rt.SDFQuality = 3;
                    if (DrawModeButton(host, "Ultra", rt.SDFQuality == 4, 42f)) rt.SDFQuality = 4;
                    GUILayout.EndHorizontal();
                }
                GUILayout.Space(2f);

                // SDF stats
                if (rt != null && rt.SDFVoxelCount > 0)
                {
                    string sdfLabel = rt.SDFVoxelCount + " voxels  /  " + rt.SDFVertexCount + " verts";
                    host.DrawStatRow("SDF", sdfLabel, host.SuccessColorRef);
                }

                GUILayout.BeginHorizontal();
                if (GUILayout.Button("Build Colliders", GUILayout.Height(24f), GUILayout.Width(114f)))
                    logic.RebuildSDFCollider(selection, GetOrCreateSDFGroups());
                if (GUILayout.Button("Clear", GUILayout.Height(24f), GUILayout.Width(48f)))
                {
                    var g = GetOrCreateSDFGroups();
                    foreach (var key in new System.Collections.Generic.List<string>(g.Keys)) g[key] = false;
                    logic.RebuildSDFCollider(selection, g);
                }
                GUILayout.EndHorizontal();
            }

            // ── Joan6694 scene colliders ───────────────────────────────────
            int joanCount = Joan6694ColliderWatcher.Colliders.Count;
            if (joanCount > 0)
            {
                GUILayout.Space(3f);
                host.DrawStatRow("Scene DB Colliders", joanCount + " active", host.SuccessColorRef);
            }
        }

        private static bool DrawModeButton(BaseUI host, string label, bool active, float width = 62f)
        {
            string txt = active ? "[" + label + "]" : " " + label + " ";
            return GUILayout.Button(txt, host.HintStyleRef, GUILayout.Width(width), GUILayout.Height(22f));
        }
    }
}
