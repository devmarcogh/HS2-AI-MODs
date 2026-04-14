using System;
using System.Collections.Generic;
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
            public float Gravity;
            public float Compression;
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
        private Vector2          listScroll = Vector2.zero;
        private Vector2          boneScroll = Vector2.zero;
        private const float      ListHeight = 170f;
        private readonly Dictionary<ClothMeshState, HashSet<string>> expandedBoneNodesByMesh =
            new Dictionary<ClothMeshState, HashSet<string>>();

        public string ModuleId => FeatureModuleIds.ClothPhysics;

        public bool GetDefaultEnabledState() => true;
        public void SyncEnabledState(ref bool enabled) { }
        public void OnToggleChanged(bool enabled) { }

        public bool TryCopyConfig(SelectionContext selection, out object config)
        {
            config = null;
            if (selectedMesh == null || selectedMesh.Params == null) return false;

            ClothPhysicsParams p = selectedMesh.Params;
            config = new ClothConfigSnapshot
            {
                StretchStiffness = p.StretchStiffness,
                BendStiffness = p.BendStiffness,
                Damping = p.Damping,
                Thickness = p.Thickness,
                Gravity = p.Gravity,
                Compression = p.Compression,
                Substeps = p.Substeps,
                Iterations = p.Iterations,
                ClothToCloth = p.ClothToCloth,
                PinSourceBoneNames = new List<string>(selectedMesh.PinSourceBoneNames ?? new List<string>()),
                ShowPinBoneGizmos = selectedMesh.ShowPinBoneGizmos,
            };
            return true;
        }

        public bool TryPasteConfig(SelectionContext selection, object config)
        {
            ClothConfigSnapshot snap = config as ClothConfigSnapshot;
            if (snap == null || selectedMesh == null || selectedMesh.Params == null) return false;

            ClothPhysicsParams p = selectedMesh.Params;
            p.StretchStiffness = snap.StretchStiffness;
            p.BendStiffness = snap.BendStiffness;
            p.Damping = snap.Damping;
            p.Thickness = snap.Thickness;
            p.Gravity = snap.Gravity;
            p.Compression = snap.Compression;
            p.Substeps = snap.Substeps;
            p.Iterations = snap.Iterations;
            p.ClothToCloth = snap.ClothToCloth;
            selectedMesh.ShowPinBoneGizmos = snap.ShowPinBoneGizmos;

            selectedMesh.PinSourceBoneNames.Clear();
            if (snap.PinSourceBoneNames != null)
            {
                for (int i = 0; i < snap.PinSourceBoneNames.Count; i++)
                    AddBone(selectedMesh.PinSourceBoneNames, snap.PinSourceBoneNames[i]);
            }

            logic.RecomputePins(selection, selectedMesh);
            return true;
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
                selectedMesh  = null;
            }

            IReadOnlyList<ClothPhysicsEntry> entries = logic.GetEntries(selection);

            if (entries == null || entries.Count == 0)
            {
                GUILayout.Label("No cloth items found on this character. Make sure clothing is equipped.", host.HintStyleRef);
                GUILayout.Space(6f);
                if (GUILayout.Button("Refresh", GUILayout.Height(26f)))
                    logic.RefreshEntries(selection);
                return;
            }

            // ── Cloth list (accordion) ─────────────────────────────────────
            listScroll = GUILayout.BeginScrollView(listScroll, GUILayout.Height(ListHeight));

            foreach (ClothPhysicsEntry entry in entries)
            {
                // Category header button
                string arrow  = entry.IsExpanded ? "▼" : "▶";
                string header = arrow + "  " + entry.DisplayName;
                if (GUILayout.Button(header, host.HintStyleRef, GUILayout.Height(24f)))
                    entry.IsExpanded = !entry.IsExpanded;

                if (!entry.IsExpanded) continue;

                GUILayout.Space(2f);
                foreach (ClothMeshState ms in entry.Meshes)
                {
                    bool isSelected = ms == selectedMesh;
                    GUILayout.BeginHorizontal();
                    GUILayout.Space(18f);

                    // Active toggle
                    bool newActive = GUILayout.Toggle(ms.IsActive, string.Empty, GUILayout.Width(18f));
                    if (newActive != ms.IsActive)
                        logic.ToggleMesh(selection, ms);

                    // Mesh name button (selects it for settings panel)
                    string label = isSelected ? "► " + ms.MeshName : "   " + ms.MeshName;
                    if (GUILayout.Button(label, host.HintStyleRef, GUILayout.ExpandWidth(true), GUILayout.Height(22f)))
                        selectedMesh = (selectedMesh == ms) ? null : ms;

                    GUILayout.EndHorizontal();
                    GUILayout.Space(2f);
                }
                GUILayout.Space(4f);
            }

            GUILayout.EndScrollView();

            // Refresh + hint
            GUILayout.Space(4f);
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Refresh list", GUILayout.Height(24f), GUILayout.Width(100f)))
            {
                selectedMesh = null;
                logic.RefreshEntries(selection);
            }
            GUILayout.Label("Toggle to activate physics · click mesh to edit settings", host.HintStyleRef);
            GUILayout.EndHorizontal();

            // ── Per-mesh settings (shown when a mesh is selected) ──────────
            if (selectedMesh == null) return;

            GUILayout.Space(8f);
            host.DrawStatRow("Editing", selectedMesh.MeshName, host.SuccessColorRef);
            host.DrawStatRow("Status",
                selectedMesh.IsActive ? "Simulating" : "Inactive",
                selectedMesh.IsActive ? host.SuccessColorRef : host.WarningColorRef);
            GUILayout.Space(6f);

            ClothPhysicsParams p = selectedMesh.Params;

            DrawFloatRow(host, "Stretch",   ref p.StretchStiffness, 10f,    2000f);
            DrawFloatRow(host, "Bending",   ref p.BendStiffness,    0f,     2f);
            DrawFloatRow(host, "Damping",   ref p.Damping,          0f,     40f);
            DrawFloatRow(host, "Thickness", ref p.Thickness,        0.002f, 0.05f);
            DrawFloatRow(host, "Gravity",      ref p.Gravity,          -30f,   0f);
            DrawFloatRow(host, "Compression", ref p.Compression,       0f,     1f);

            GUILayout.Space(4f);
            DrawPresetRow(host, "Substeps",   ref p.Substeps);
            DrawPresetRow(host, "Iterations", ref p.Iterations);

            GUILayout.Space(4f);
            GUILayout.BeginHorizontal();
            GUILayout.Label("Cloth-to-cloth collision", host.HintStyleRef, GUILayout.Width(180f));
            p.ClothToCloth = GUILayout.Toggle(p.ClothToCloth, p.ClothToCloth ? "On" : "Off", GUILayout.Width(40f));
            GUILayout.EndHorizontal();

            // ── Pinning by source bones (treeview) ───────────────────────────────────
            GUILayout.Space(8f);
            GUILayout.Label("Pin Source Bones  (dominant vertices become pinned)", host.HintStyleRef);
            GUILayout.Space(2f);

            DrawBoneTree(host, selectedMesh);

            GUILayout.Space(4f);
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Select All", GUILayout.Height(24f), GUILayout.Width(90f)))
                SelectAllBones(selectedMesh);
            if (GUILayout.Button("Unselect All", GUILayout.Height(24f), GUILayout.Width(90f)))
                selectedMesh.PinSourceBoneNames.Clear();
            selectedMesh.ShowPinBoneGizmos = GUILayout.Toggle(selectedMesh.ShowPinBoneGizmos, "Show Gizmos", GUILayout.Width(100f));
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Apply Pins", GUILayout.Height(26f), GUILayout.Width(90f)))
                logic.RecomputePins(lastSelection, selectedMesh);
            GUILayout.EndHorizontal();

            int pinned = 0;
            if (selectedMesh.IsPinned != null)
                for (int i = 0; i < selectedMesh.IsPinned.Length; i++)
                    if (selectedMesh.IsPinned[i]) pinned++;

            host.DrawStatRow("Selected Bones", selectedMesh.PinSourceBoneNames.Count.ToString(), host.HintStyleRef.normal.textColor);
            host.DrawStatRow("Pinned Verts", pinned.ToString(), host.HintStyleRef.normal.textColor);
        }

        // ------------------------------------------------------------------ //
        // Helpers
        // ------------------------------------------------------------------ //

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
            float[] options = { 0.25f, 0.5f, 0.75f, 1f };
            GUILayout.BeginHorizontal();
            GUILayout.Label(label, host.HintStyleRef, GUILayout.Width(100f));
            for (int i = 0; i < options.Length; i++)
            {
                float v = options[i];
                bool selected = Mathf.Abs(value - v) <= 0.0001f;
                string txt = v.ToString("0.##");
                string btnTxt = selected ? "[" + txt + "]" : " " + txt + " ";
                if (GUILayout.Button(btnTxt, host.HintStyleRef, GUILayout.Width(30f), GUILayout.Height(22f)))
                    value = v;
            }
            GUILayout.EndHorizontal();
        }
    }
}
