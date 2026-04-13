using System.Collections.Generic;
using UnityEngine;

namespace StudioModsMSG
{
    /// <summary>
    /// GUI panel for the Cloth Physics module.
    ///
    /// Layout:
    ///   ┌─ scroll view ───────────────────────────────────────────────────── ┐
    ///   │  [▶ Top]                        (category header — click to expand)│
    ///   │      [x] o_top_a                                                   │
    ///   │      [ ] o_top_b_cf_1           (mesh toggle)                      │
    ///   │  [▼ Bottom]                                                        │
    ///   │      [x] o_bot_a      ← selected mesh (bold)                       │
    ///   └─────────────────────────────────────────────────────────────────── ┘
    ///   ┌─ selected mesh settings (shown only when a mesh is selected) ────── ┐
    ///   │   Stretch     ███████░░░ 600                                        │
    ///   │   Bending     ██░░░░░░░░ 0.3                                        │
    ///   │   Damping     ████░░░░░░ 5                                          │
    ///   │   Thickness   █░░░░░░░░░ 0.012                                      │
    ///   │   Gravity     ████░░░░░░ -9.8                                       │
    ///   │   Substeps    1  [2]  3                                             │
    ///   │   Iterations  1  2  [3]                                             │
    ///   │   [x] Cloth-to-cloth collision                                      │
    ///   └─────────────────────────────────────────────────────────────────── ┘
    /// </summary>
    class ClothPhysicsPanel : IModulePanelUI
    {
        private readonly ClothPhysicsModule logic = new ClothPhysicsModule();

        // GUI state
        private SelectionContext lastSelection;
        private ClothMeshState   selectedMesh;
        private Vector2          listScroll    = Vector2.zero;
        private Vector2          settingsScroll = Vector2.zero;
        private const float ListHeight         = 180f;

        public string ModuleId => FeatureModuleIds.ClothPhysics;

        public bool GetDefaultEnabledState() => true;
        public void SyncEnabledState(ref bool enabled) { }
        public void OnToggleChanged(bool enabled) { }

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
                    GUIStyle btnStyle = isSelected ? host.HintStyleRef : host.HintStyleRef;
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

            settingsScroll = GUILayout.BeginScrollView(settingsScroll, GUILayout.ExpandHeight(false));

            DrawFloatRow(host, "Stretch",   ref p.StretchStiffness, 10f,    2000f);
            DrawFloatRow(host, "Bending",   ref p.BendStiffness,    0f,     2f);
            DrawFloatRow(host, "Damping",   ref p.Damping,          0f,     40f);
            DrawFloatRow(host, "Thickness", ref p.Thickness,        0.002f, 0.05f);
            DrawFloatRow(host, "Gravity",      ref p.Gravity,          -30f,   0f);
            DrawFloatRow(host, "Compression", ref p.Compression,       0f,     1f);

            GUILayout.Space(4f);
            DrawIntRow(host, "Substeps",   ref p.Substeps,   1, 4);
            DrawIntRow(host, "Iterations", ref p.Iterations, 1, 6);

            GUILayout.Space(4f);
            GUILayout.BeginHorizontal();
            GUILayout.Label("Cloth-to-cloth collision", host.HintStyleRef, GUILayout.Width(180f));
            p.ClothToCloth = GUILayout.Toggle(p.ClothToCloth, p.ClothToCloth ? "On" : "Off", GUILayout.Width(40f));
            GUILayout.EndHorizontal();

            GUILayout.EndScrollView();
        }

        // ------------------------------------------------------------------ //
        // Helpers
        // ------------------------------------------------------------------ //
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

        private static void DrawIntRow(BaseUI host, string label, ref int value, int min, int max)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label(label, host.HintStyleRef, GUILayout.Width(100f));
            for (int v = min; v <= max; v++)
            {
                bool selected = (value == v);
                string btnTxt = selected ? "[" + v + "]" : " " + v + " ";
                if (GUILayout.Button(btnTxt, host.HintStyleRef, GUILayout.Width(30f), GUILayout.Height(22f)))
                    value = v;
            }
            GUILayout.EndHorizontal();
        }
    }
}
