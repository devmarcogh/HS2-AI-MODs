using Studio;
using System;
using System.Collections.Generic;
using System.Linq;
using BepInEx.Configuration;
using UnityEngine;

namespace StudioModsMSG
{
    class BaseUI : MonoBehaviour
    {
        private const float HeaderHeight = 58f;
        private const float OuterPadding = 14f;
        private const float SectionGap = 10f;
        private bool? lastAppliedDarkMode;

        private readonly int windowID = 10123;
        private readonly string windowTitle = StudioCharaEditor.Name;
        private Rect windowRect = new Rect(0f, 300f, 640f, 360f);
        private bool mouseInWindow = false;
        private readonly float minWindowWidth = 520f;
        private readonly float minWindowHeight = 280f;
        private readonly float resizeHandleSize = 16f;
        private readonly float resizeHitSize = 28f;
        private bool isResizing = false;
        private Vector2 resizeStartMousePos;
        private Vector2 resizeStartSize;
        private int resizeControlId = 0;
        private GUIStyle windowStyle;
        private GUIStyle headerStyle;
        private GUIStyle titleStyle;
        private GUIStyle subtitleStyle;
        private GUIStyle cardStyle;
        private GUIStyle labelStyle;
        private GUIStyle valueStyle;
        private GUIStyle sectionTitleStyle;
        private GUIStyle chipStyle;
        private GUIStyle hintStyle;
        private GUIStyle buttonStyle;
        private GUIStyle drawerStyle;
        private GUIStyle drawerItemStyle;
        private GUIStyle drawerItemSelectedStyle;
        private GUIStyle moduleToggleStyle;
        private GUIStyle rowLabelStyle;
        private GUIStyle rowValueStyle;
        private GUIStyle closeButtonStyle;
        private GUIStyle emptyStateStyle;
        private GUIStyle verticalScrollbarStyle;
        private GUIStyle verticalScrollbarThumbStyle;
        private GUIStyle horizontalScrollbarStyle;
        private GUIStyle horizontalScrollbarThumbStyle;
        private GUIStyle scrollViewBackgroundStyle;
        private GUIStyle horizontalSliderStyle;
        private GUIStyle horizontalSliderThumbStyle;
        private Texture2D windowBackgroundTexture;
        private Texture2D headerBackgroundTexture;
        private Texture2D cardBackgroundTexture;
        private Texture2D accentTexture;
        private Texture2D chipTexture;
        private Texture2D closeTexture;
        private Texture2D borderTexture;
        private Texture2D drawerBackgroundTexture;
        private Texture2D drawerItemTexture;
        private Texture2D drawerItemSelectedTexture;
        private Texture2D toggleOffTexture;
        private Texture2D toggleOnTexture;
        private Texture2D sliderTrackTexture;
        private Texture2D sliderThumbTexture;
        private Texture2D scrollTrackTexture;
        private Texture2D scrollThumbTexture;

        private TreeNodeObject lastSelectedTreeNode;
        private OCIChar ociTarget;
        private ObjectCtrlInfo selectedTarget;
        private readonly List<string> compatibleModuleNames = new List<string>();
        private readonly Dictionary<string, bool> moduleToggleStates = new Dictionary<string, bool>();
        private readonly Dictionary<string, IModulePanelUI> modulePanels = new Dictionary<string, IModulePanelUI>(StringComparer.OrdinalIgnoreCase);
        private string selectedModuleId;
        private Vector2 moduleDrawerScroll = Vector2.zero;

        private bool IsDarkMode => StudioCharaEditor.UIDarkMode != null && StudioCharaEditor.UIDarkMode.Value;
        private Color WindowBackgroundColor => IsDarkMode ? Color.black : Color.white;
        private Color HeaderBackgroundColor => IsDarkMode ? Color.black : Color.white;
        private Color CardBackgroundColor => IsDarkMode ? Color.black : Color.white;
        private Color AccentColor => IsDarkMode ? Color.black : Color.white;
        private Color AccentSoftColor => IsDarkMode ? Color.white : Color.black;
        private Color SuccessColor => IsDarkMode ? Color.white : Color.black;
        private Color WarningColor => IsDarkMode ? Color.white : Color.black;
        private Color TextPrimaryColor => IsDarkMode ? Color.white : Color.black;
        private Color TextSecondaryColor => IsDarkMode ? Color.white : Color.black;
        private Color BorderColor => IsDarkMode ? Color.white : Color.black;
        private Color DrawerBackgroundColor => IsDarkMode ? Color.black : Color.white;
        private Color DrawerItemColor => IsDarkMode ? Color.black : Color.white;
        private Color DrawerItemSelectedColor => IsDarkMode ? Color.white : Color.black;
        private Color ToggleOffColor => IsDarkMode ? Color.black : Color.white;
        private Color ToggleOnColor => IsDarkMode ? Color.white : Color.black;
        private Color SliderTrackColor => IsDarkMode ? Color.black : Color.white;
        private Color SliderThumbColor => IsDarkMode ? Color.white : Color.black;
        private Color ScrollTrackColor => IsDarkMode ? Color.black : Color.white;
        private Color ScrollThumbColor => IsDarkMode ? Color.white : Color.black;

        public static Queue<Action> ToDoQueue = new Queue<Action>();

        public bool VisibleGUI { get; set; }

        public void ResetGui()
        {
            ociTarget = null;
            selectedTarget = null;
            compatibleModuleNames.Clear();
            moduleToggleStates.Clear();
            selectedModuleId = null;
            moduleDrawerScroll = Vector2.zero;
        }

        private void Start()
        {
            windowRect = new Rect(
                StudioCharaEditor.UIXPosition.Value,
                StudioCharaEditor.UIYPosition.Value,
                Math.Max(minWindowWidth, StudioCharaEditor.UIWidth.Value),
                Math.Max(minWindowHeight, StudioCharaEditor.UIHeight.Value)
            );

            EnsureStyles();
            EnsureModulePanels();
        }

        private void OnGUI()
        {
            if (VisibleGUI)
            {
                try
                {
                    if (!lastAppliedDarkMode.HasValue || lastAppliedDarkMode.Value != IsDarkMode)
                    {
                        ResetStyles();
                    }

                    EnsureStyles();

                    DrawWindowBackdrop();

                    windowRect = GUI.Window(windowID, windowRect, new GUI.WindowFunction(FuncWindowGUI), string.Empty, windowStyle);


                    mouseInWindow = windowRect.Contains(Event.current.mousePosition);
                    if (mouseInWindow)
                    {
                        Studio.Studio.Instance.cameraCtrl.noCtrlCondition = (() => mouseInWindow && VisibleGUI);
                        Input.ResetInputAxes();
                    }
                }
                catch (Exception ex)
                {
                    Console.WriteLine(ex);
                }
            }
        }

        private void DrawWindowBackdrop()
        {
        }

        private void Update()
        {
            if (StudioCharaEditor.KeyShowUI.Value.IsDown())
            {
                VisibleGUI = !VisibleGUI;
                if (VisibleGUI)
                {
                    windowRect = new Rect(StudioCharaEditor.UIXPosition.Value, StudioCharaEditor.UIYPosition.Value, Math.Max(minWindowWidth, StudioCharaEditor.UIWidth.Value), Math.Max(minWindowHeight, StudioCharaEditor.UIHeight.Value));
                }
                else
                {
                    StudioCharaEditor.UIXPosition.Value = (int)windowRect.x;
                    StudioCharaEditor.UIYPosition.Value = (int)windowRect.y;
                    StudioCharaEditor.UIWidth.Value = (int)windowRect.width;
                    StudioCharaEditor.UIHeight.Value = (int)windowRect.height;
                }
            }

            if (VisibleGUI)
            {
                TreeNodeObject curSel = Studio.Studio.Instance.treeNodeCtrl.selectNode;
                if (curSel != lastSelectedTreeNode)
                {
                    OnSelectChange(curSel);
                }
            }

            CharaEditorMgr.Instance.HouseKeeping(VisibleGUI);

            if (ToDoQueue.Count > 0)
            {
                Action p = ToDoQueue.Dequeue();
                p();
            }
        }

        private void FuncWindowGUI(int winID)
        {
            // Track whether each layout-group scope was opened so that the catch
            // block can close them before returning.  Unclosed BeginArea /
            // BeginVertical groups corrupt the IMGUI layout stack for the entire
            // frame and cause GUILayoutUtility.GetRect exceptions in every other
            // plugin that calls FlexibleSpace() afterwards.
            bool headerAreaOpen  = false;
            bool contentAreaOpen = false;
            try
            {
                HandleWindowResize();
                SelectionContext sel = CharaEditorMgr.Instance.ActiveSelection;

                DrawGlassWindowShell();

                if (Event.current.type == EventType.MouseDown)
                {
                    GUI.FocusControl("");
                    GUI.FocusWindow(winID);
                }

                // DrawHeader opens/closes its own BeginArea — guard it separately.
                headerAreaOpen = true;
                DrawHeader(sel);
                headerAreaOpen = false;

                Rect contentRect = new Rect(OuterPadding, HeaderHeight + OuterPadding - 4f, windowRect.width - (OuterPadding * 2f), windowRect.height - HeaderHeight - (OuterPadding * 2f) - 10f);
                GUILayout.BeginArea(contentRect);
                contentAreaOpen = true;
                GUILayout.BeginVertical();
                if (!sel.HasSelection)
                {
                    DrawEmptyState();
                }
                else
                {
                    DrawSelectionSummary(sel);
                    GUILayout.Space(SectionGap);
                    DrawModuleWorkspace(sel);
                }
                GUILayout.EndVertical();
                GUILayout.EndArea();
                contentAreaOpen = false;

                Rect closeButtonRect = new Rect(windowRect.width - 38f, 14f, 24f, 24f);
                if (GUI.Button(closeButtonRect, "x", closeButtonStyle))
                {
                    VisibleGUI = false;
                }

                if (!isResizing)
                {
                    GUI.DragWindow(new Rect(0, 0, windowRect.width - resizeHitSize - 40f, HeaderHeight));
                }
                DrawResizeHandle();
            }
            catch (Exception ex)
            {
                // Close any IMGUI layout groups that were left open by the exception.
                // Not doing so corrupts the layout stack for all subsequent OnGUI calls
                // in this frame (including other BepInEx plugins).
                if (contentAreaOpen)
                {
                    try { GUILayout.EndVertical(); } catch { }
                    try { GUILayout.EndArea();     } catch { }
                }
                if (headerAreaOpen)
                {
                    // DrawHeader itself uses BeginArea/EndArea internally; if it threw
                    // mid-way those groups are already closed by Unity's window machinery,
                    // so we only need to end groups we actually opened above.
                    // (headerAreaOpen just marks that DrawHeader itself threw.)
                }
                UnityEngine.Debug.LogError("[StudioCharaMods] FuncWindowGUI exception: " + ex);
                ResetGui();
            }
        }

        private void DrawGlassWindowShell()
        {
            Rect fullRect = new Rect(0f, 0f, windowRect.width, windowRect.height);
            GUI.DrawTexture(fullRect, windowBackgroundTexture, ScaleMode.StretchToFill);
            DrawFlatBorder(fullRect, 1f);
        }


        private void DrawResizeHandle()
        {
            Rect handleRect = new Rect(windowRect.width - resizeHandleSize - 2f, windowRect.height - resizeHandleSize - 2f, resizeHandleSize, resizeHandleSize);
            GUI.Label(handleRect, "///", hintStyle);
        }

        private void DrawHeader(SelectionContext selection)
        {
            GUI.DrawTexture(new Rect(0f, 0f, windowRect.width, HeaderHeight), headerBackgroundTexture, ScaleMode.StretchToFill);
            GUI.DrawTexture(new Rect(0f, HeaderHeight - 1f, windowRect.width, 1f), borderTexture, ScaleMode.StretchToFill);
            DrawFlatBorder(new Rect(0f, 0f, windowRect.width, HeaderHeight), 1f);

            bool areaOpen = false;
            try
            {
                GUILayout.BeginArea(new Rect(14f, 10f, windowRect.width - 60f, HeaderHeight - 12f));
                areaOpen = true;
                GUILayout.BeginHorizontal();
                GUILayout.BeginVertical();
                GUILayout.FlexibleSpace();
                GUILayout.Label(windowTitle, titleStyle);
                GUILayout.FlexibleSpace();
                GUILayout.EndVertical();
                GUILayout.FlexibleSpace();
                if (GUILayout.Button(IsDarkMode ? "Night" : "Day", buttonStyle, GUILayout.Width(74f), GUILayout.Height(26f)))
                {
                    StudioCharaEditor.UIDarkMode.Value = !StudioCharaEditor.UIDarkMode.Value;
                    ResetStyles();
                }
                GUILayout.EndHorizontal();
                GUILayout.EndArea();
                areaOpen = false;
            }
            catch (Exception ex)
            {
                if (areaOpen)
                {
                    try { GUILayout.EndHorizontal(); } catch { }
                    try { GUILayout.EndArea();       } catch { }
                }
                UnityEngine.Debug.LogError("[StudioCharaMods] DrawHeader exception: " + ex);
            }
        }

        private void DrawEmptyState()
        {
            Rect cardRect = BeginGlassCard();
            GUILayout.Space(8f);
            GUILayout.Label("No active selection", sectionTitleStyle);
            GUILayout.Space(6f);
            GUILayout.Label("Pick a character or compatible object in Studio. The editor will load the available modules automatically.", emptyStateStyle);
            GUILayout.Space(12f);
            GUILayout.Label("Tip: use the shortcut shown in the top-right corner to hide or reopen this panel.", hintStyle);
            GUILayout.Space(8f);
            EndGlassCard(cardRect);
        }

        private void DrawSelectionSummary(SelectionContext selection)
        {
            Rect cardRect = BeginGlassCard();
            GUILayout.Label("Selection Overview", sectionTitleStyle);
            GUILayout.Space(8f);
            DrawStatRow("Target", selection.TargetTypeName, AccentColor);
            DrawStatRow("Modules", compatibleModuleNames.Count.ToString(), SuccessColor);
            DrawStatRow("State", compatibleModuleNames.Count > 0 ? "Ready" : "No compatible modules", compatibleModuleNames.Count > 0 ? SuccessColor : WarningColor);
            GUILayout.Space(10f);
            GUILayout.Label("The panel is docked as an overlay. Drag the header to move it and use the lower-right corner to resize.", hintStyle);
            EndGlassCard(cardRect);
        }

        private void DrawModuleWorkspace(SelectionContext selection)
        {
            EnsureModuleUiState(selection);

            Rect cardRect = BeginGlassCard();
            GUILayout.BeginHorizontal();

            DrawModuleNavigationDrawer(selection);
            GUILayout.Space(10f);
            DrawSelectedModulePanel(selection);

            GUILayout.EndHorizontal();
            EndGlassCard(cardRect);
        }

        private void DrawModuleNavigationDrawer(SelectionContext selection)
        {
            const float drawerWidth = 224f;

            GUILayout.BeginVertical(drawerStyle, GUILayout.Width(drawerWidth), GUILayout.ExpandHeight(true));
            GUILayout.Label("Modules", sectionTitleStyle);
            GUILayout.Space(6f);

            moduleDrawerScroll = GUILayout.BeginScrollView(
                moduleDrawerScroll,
                false,
                true,
                horizontalScrollbarStyle,
                verticalScrollbarStyle,
                scrollViewBackgroundStyle,
                GUILayout.ExpandHeight(true));

            foreach (IEditorModule module in selection.CompatibleModules)
            {
                bool isSelected = string.Equals(selectedModuleId, module.ModuleId, StringComparison.OrdinalIgnoreCase);
                if (!moduleToggleStates.ContainsKey(module.ModuleId))
                {
                    moduleToggleStates[module.ModuleId] = true;
                }

                GUILayout.BeginHorizontal(isSelected ? drawerItemSelectedStyle : drawerItemStyle, GUILayout.Height(34f));

                bool isEnabled = moduleToggleStates[module.ModuleId];
                bool newEnabled = DrawModuleToggle(isEnabled);
                if (newEnabled != isEnabled)
                {
                    moduleToggleStates[module.ModuleId] = newEnabled;
                    OnModuleToggleChanged(module.ModuleId, newEnabled);
                }

                if (GUILayout.Button(module.DisplayName, isSelected ? drawerItemSelectedStyle : drawerItemStyle, GUILayout.Height(28f), GUILayout.ExpandWidth(true)))
                {
                    selectedModuleId = module.ModuleId;
                }

                GUILayout.EndHorizontal();
                GUILayout.Space(6f);
            }

            GUILayout.EndScrollView();
            GUILayout.EndVertical();
        }

        private void DrawSelectedModulePanel(SelectionContext selection)
        {
            GUILayout.BeginVertical(GUILayout.ExpandWidth(true), GUILayout.ExpandHeight(true));

            IEditorModule selectedModule = selection.CompatibleModules.FirstOrDefault(module => string.Equals(module.ModuleId, selectedModuleId, StringComparison.OrdinalIgnoreCase));
            if (selectedModule == null)
            {
                GUILayout.Label("Select a module from the left drawer.", emptyStateStyle);
                GUILayout.EndVertical();
                return;
            }

            bool enabled = moduleToggleStates.ContainsKey(selectedModule.ModuleId) && moduleToggleStates[selectedModule.ModuleId];
            GUILayout.Label(selectedModule.DisplayName, sectionTitleStyle);
            GUILayout.Space(6f);
            DrawStatRow("Status", enabled ? "Enabled" : "Disabled", enabled ? SuccessColor : WarningColor);
            GUILayout.Space(8f);

            if (!enabled)
            {
                GUILayout.Label("This module is turned off. Enable it from the drawer switch to edit its configuration.", hintStyle);
                GUILayout.EndVertical();
                return;
            }

            DrawModuleConfiguration(selection, selectedModule.ModuleId);
            GUILayout.EndVertical();
        }

        private void DrawModuleConfiguration(SelectionContext selection, string moduleId)
        {
            EnsureModulePanels();

            if (modulePanels.ContainsKey(moduleId))
            {
                modulePanels[moduleId].Draw(selection, this);
                return;
            }

            GUILayout.Label("No UI panel is registered for this module yet.", hintStyle);
        }

        internal void DrawFloatSliderSetting(string label, ConfigEntry<float> entry, float minValue, float maxValue)
        {
            Rect rowRect = GUILayoutUtility.GetRect(10f, 40f, GUILayout.ExpandWidth(true));
            GUI.DrawTexture(new Rect(rowRect.x, rowRect.y + 2f, rowRect.width, rowRect.height - 4f), chipTexture, ScaleMode.StretchToFill);
            DrawFlatBorder(new Rect(rowRect.x, rowRect.y + 2f, rowRect.width, rowRect.height - 4f), 1f);

            GUI.Label(new Rect(rowRect.x + 10f, rowRect.y + 6f, rowRect.width - 110f, 18f), label, rowLabelStyle);
            GUI.Label(new Rect(rowRect.x + rowRect.width - 94f, rowRect.y + 6f, 84f, 18f), entry.Value.ToString("0.###"), rowValueStyle);

            Rect sliderRect = new Rect(rowRect.x + 10f, rowRect.y + 24f, rowRect.width - 20f, 14f);
            float newValue = GUI.HorizontalSlider(sliderRect, entry.Value, minValue, maxValue, horizontalSliderStyle, horizontalSliderThumbStyle);
            if (Mathf.Abs(newValue - entry.Value) > 0.0001f)
            {
                entry.Value = newValue;
            }
        }

        private void EnsureModuleUiState(SelectionContext selection)
        {
            HashSet<string> validIds = new HashSet<string>(selection.CompatibleModules.Select(module => module.ModuleId), StringComparer.OrdinalIgnoreCase);
            List<string> staleIds = moduleToggleStates.Keys.Where(id => !validIds.Contains(id)).ToList();
            foreach (string staleId in staleIds)
            {
                moduleToggleStates.Remove(staleId);
            }

            foreach (IEditorModule module in selection.CompatibleModules)
            {
                if (!moduleToggleStates.ContainsKey(module.ModuleId))
                {
                    if (modulePanels.ContainsKey(module.ModuleId))
                    {
                        moduleToggleStates[module.ModuleId] = modulePanels[module.ModuleId].GetDefaultEnabledState();
                    }
                    else
                    {
                        moduleToggleStates[module.ModuleId] = true;
                    }
                }
            }

            foreach (IEditorModule module in selection.CompatibleModules)
            {
                if (modulePanels.ContainsKey(module.ModuleId) && moduleToggleStates.ContainsKey(module.ModuleId))
                {
                    bool current = moduleToggleStates[module.ModuleId];
                    modulePanels[module.ModuleId].SyncEnabledState(ref current);
                    moduleToggleStates[module.ModuleId] = current;
                }
            }

            if (string.IsNullOrEmpty(selectedModuleId) || !validIds.Contains(selectedModuleId))
            {
                selectedModuleId = selection.CompatibleModules.Count > 0 ? selection.CompatibleModules[0].ModuleId : null;
            }
        }

        private void OnModuleToggleChanged(string moduleId, bool enabled)
        {
            if (modulePanels.ContainsKey(moduleId))
            {
                modulePanels[moduleId].OnToggleChanged(enabled);
            }
        }

        private void EnsureModulePanels()
        {
            foreach (IModulePanelUI panel in ModulePanelFactory.CreateDefaultPanels())
            {
                if (!modulePanels.ContainsKey(panel.ModuleId))
                {
                    modulePanels[panel.ModuleId] = panel;
                }
            }
        }

        internal GUIStyle HintStyleRef => hintStyle;
        internal Color SuccessColorRef => SuccessColor;
        internal Color WarningColorRef => WarningColor;

        internal void DrawStatRow(string label, string value, Color accent)
        {
            DrawStatRowInternal(label, value, accent);
        }

        private void DrawStatRowInternal(string label, string value, Color accent)
        {
            Rect rowRect = GUILayoutUtility.GetRect(10f, 28f, GUILayout.ExpandWidth(true));
            GUI.DrawTexture(new Rect(rowRect.x, rowRect.y + 2f, rowRect.width, rowRect.height - 4f), chipTexture, ScaleMode.StretchToFill);
            GUI.Label(new Rect(rowRect.x + 18f, rowRect.y + 4f, 120f, 22f), label, rowLabelStyle);
            GUI.Label(new Rect(rowRect.x + 130f, rowRect.y + 4f, rowRect.width - 140f, 22f), value, rowValueStyle);
            DrawFlatBorder(new Rect(rowRect.x, rowRect.y + 2f, rowRect.width, rowRect.height - 4f), 1f);
        }

        private bool DrawModuleToggle(bool isEnabled)
        {
            Rect rect = GUILayoutUtility.GetRect(22f, 22f, GUILayout.Width(22f), GUILayout.Height(22f));
            bool newValue = GUI.Toggle(rect, isEnabled, isEnabled ? "X" : string.Empty, moduleToggleStyle);
            DrawFlatBorder(rect, 1f);
            return newValue;
        }

        private Rect BeginGlassCard()
        {
            GUILayout.BeginVertical(cardStyle);
            return Rect.zero;
        }

        private void EndGlassCard(Rect fallbackRect)
        {
            GUILayout.EndVertical();
            Rect cardRect = GUILayoutUtility.GetLastRect();
            if (cardRect.width > 0f && cardRect.height > 0f)
            {
                DrawFlatBorder(cardRect, 1f);
            }
        }

        private void DrawFlatBorder(Rect rect, float thickness)
        {
            GUI.DrawTexture(new Rect(rect.x, rect.y, rect.width, thickness), borderTexture, ScaleMode.StretchToFill);
            GUI.DrawTexture(new Rect(rect.x, rect.yMax - thickness, rect.width, thickness), borderTexture, ScaleMode.StretchToFill);
            GUI.DrawTexture(new Rect(rect.x, rect.y, thickness, rect.height), borderTexture, ScaleMode.StretchToFill);
            GUI.DrawTexture(new Rect(rect.xMax - thickness, rect.y, thickness, rect.height), borderTexture, ScaleMode.StretchToFill);
        }

        private string GetShortcutText()
        {
            KeyboardShortcut shortcut = StudioCharaEditor.KeyShowUI.Value;
            string modifiers = string.Join(" + ", shortcut.Modifiers);
            if (string.IsNullOrEmpty(modifiers))
            {
                return shortcut.MainKey.ToString();
            }

            return modifiers + " + " + shortcut.MainKey;
        }

        private void EnsureStyles()
        {
            if (windowStyle != null)
            {
                return;
            }

            lastAppliedDarkMode = IsDarkMode;

            float opacity = UIOpacity;

            windowBackgroundTexture = MakeColorTexture(WithAlpha(WindowBackgroundColor, opacity * 0.95f));
            headerBackgroundTexture = MakeColorTexture(WithAlpha(HeaderBackgroundColor, opacity));
            cardBackgroundTexture   = MakeColorTexture(WithAlpha(CardBackgroundColor,   opacity * 0.85f));
            accentTexture           = MakeColorTexture(AccentColor);
            chipTexture             = MakeColorTexture(AccentSoftColor);
            closeTexture            = MakeColorTexture(IsDarkMode ? Color.white : Color.black);
            borderTexture           = MakeColorTexture(BorderColor);
            drawerBackgroundTexture = MakeColorTexture(WithAlpha(DrawerBackgroundColor, opacity * 0.80f));
            drawerItemTexture         = MakeColorTexture(DrawerItemColor);
            drawerItemSelectedTexture = MakeColorTexture(DrawerItemSelectedColor);
            toggleOffTexture          = MakeColorTexture(ToggleOffColor);
            toggleOnTexture           = MakeColorTexture(ToggleOnColor);
            sliderTrackTexture        = MakeColorTexture(SliderTrackColor);
            sliderThumbTexture        = MakeColorTexture(SliderThumbColor);
            scrollTrackTexture        = MakeColorTexture(ScrollTrackColor);
            scrollThumbTexture        = MakeColorTexture(ScrollThumbColor);

            windowStyle = new GUIStyle(GUI.skin.window);
            windowStyle.normal.background = windowBackgroundTexture;
            windowStyle.hover.background = windowBackgroundTexture;
            windowStyle.active.background = windowBackgroundTexture;
            windowStyle.border = new RectOffset(12, 12, 12, 12);
            windowStyle.padding = new RectOffset(0, 0, 0, 0);

            headerStyle = new GUIStyle();
            headerStyle.normal.background = headerBackgroundTexture;

            titleStyle = new GUIStyle(GUI.skin.label);
            titleStyle.fontSize = 20;
            titleStyle.fontStyle = FontStyle.Bold;
            titleStyle.normal.textColor = TextPrimaryColor;

            subtitleStyle = new GUIStyle(GUI.skin.label);
            subtitleStyle.fontSize = 11;
            subtitleStyle.normal.textColor = TextSecondaryColor;

            cardStyle = new GUIStyle(GUI.skin.box);
            cardStyle.normal.background = cardBackgroundTexture;
            cardStyle.hover.background = cardBackgroundTexture;
            cardStyle.normal.textColor = TextPrimaryColor;
            cardStyle.border = new RectOffset(10, 10, 10, 10);
            cardStyle.padding = new RectOffset(16, 16, 14, 14);
            cardStyle.margin = new RectOffset(0, 0, 0, 0);

            sectionTitleStyle = new GUIStyle(GUI.skin.label);
            sectionTitleStyle.fontSize = 14;
            sectionTitleStyle.fontStyle = FontStyle.Bold;
            sectionTitleStyle.normal.textColor = TextPrimaryColor;

            labelStyle = new GUIStyle(GUI.skin.label);
            labelStyle.fontSize = 11;
            labelStyle.normal.textColor = TextSecondaryColor;

            valueStyle = new GUIStyle(GUI.skin.label);
            valueStyle.fontSize = 12;
            valueStyle.fontStyle = FontStyle.Bold;
            valueStyle.normal.textColor = TextPrimaryColor;

            rowLabelStyle = new GUIStyle(labelStyle);
            rowLabelStyle.normal.textColor = IsDarkMode ? Color.black : Color.white;

            rowValueStyle = new GUIStyle(valueStyle);
            rowValueStyle.normal.textColor = IsDarkMode ? Color.black : Color.white;

            chipStyle = new GUIStyle(GUI.skin.label);
            chipStyle.normal.background = chipTexture;
            chipStyle.hover.background = chipTexture;
            chipStyle.normal.textColor = AccentColor;
            chipStyle.alignment = TextAnchor.MiddleCenter;
            chipStyle.padding = new RectOffset(12, 12, 4, 4);
            chipStyle.margin = new RectOffset(0, 8, 0, 0);
            chipStyle.fontSize = 11;
            chipStyle.fontStyle = FontStyle.Bold;

            hintStyle = new GUIStyle(GUI.skin.label);
            hintStyle.fontSize = 11;
            hintStyle.wordWrap = true;
            hintStyle.normal.textColor = TextSecondaryColor;

            emptyStateStyle = new GUIStyle(hintStyle);
            emptyStateStyle.fontSize = 13;
            emptyStateStyle.normal.textColor = TextPrimaryColor;

            buttonStyle = new GUIStyle(GUI.skin.button);
            buttonStyle.normal.background = chipTexture;
            buttonStyle.normal.textColor = AccentColor;
            buttonStyle.hover.background = chipTexture;
            buttonStyle.hover.textColor = AccentColor;
            buttonStyle.active.background = chipTexture;
            buttonStyle.active.textColor = AccentColor;
            buttonStyle.fontStyle = FontStyle.Bold;
            buttonStyle.border = new RectOffset(8, 8, 8, 8);

            headerStyle = new GUIStyle(GUI.skin.box);
            headerStyle.normal.background = headerBackgroundTexture;
            headerStyle.normal.textColor = TextPrimaryColor;

            drawerStyle = new GUIStyle(GUI.skin.box);
            drawerStyle.normal.background = drawerBackgroundTexture;
            drawerStyle.hover.background = drawerBackgroundTexture;
            drawerStyle.normal.textColor = TextPrimaryColor;
            drawerStyle.padding = new RectOffset(10, 10, 10, 10);
            drawerStyle.margin = new RectOffset(0, 0, 0, 0);
            drawerStyle.border = new RectOffset(10, 10, 10, 10);

            drawerItemStyle = new GUIStyle(GUI.skin.button);
            drawerItemStyle.normal.background = drawerItemTexture;
            drawerItemStyle.hover.background = drawerItemTexture;
            drawerItemStyle.active.background = drawerItemSelectedTexture;
            drawerItemStyle.normal.textColor = TextPrimaryColor;
            drawerItemStyle.hover.textColor = TextPrimaryColor;
            drawerItemStyle.active.textColor = IsDarkMode ? Color.black : Color.white;
            drawerItemStyle.alignment = TextAnchor.MiddleLeft;
            drawerItemStyle.padding = new RectOffset(8, 8, 4, 4);
            drawerItemStyle.border = new RectOffset(8, 8, 8, 8);

            drawerItemSelectedStyle = new GUIStyle(drawerItemStyle);
            drawerItemSelectedStyle.normal.background = drawerItemSelectedTexture;
            drawerItemSelectedStyle.hover.background = drawerItemSelectedTexture;
            drawerItemSelectedStyle.active.background = drawerItemSelectedTexture;
            drawerItemSelectedStyle.normal.textColor = IsDarkMode ? Color.black : Color.white;
            drawerItemSelectedStyle.hover.textColor = IsDarkMode ? Color.black : Color.white;
            drawerItemSelectedStyle.active.textColor = IsDarkMode ? Color.black : Color.white;
            drawerItemSelectedStyle.fontStyle = FontStyle.Bold;

            moduleToggleStyle = new GUIStyle(GUI.skin.toggle);
            moduleToggleStyle.normal.background = toggleOffTexture;
            moduleToggleStyle.onNormal.background = toggleOnTexture;
            moduleToggleStyle.hover.background = toggleOffTexture;
            moduleToggleStyle.onHover.background = toggleOnTexture;
            moduleToggleStyle.active.background = toggleOffTexture;
            moduleToggleStyle.onActive.background = toggleOnTexture;
            moduleToggleStyle.normal.textColor = IsDarkMode ? Color.white : Color.black;
            moduleToggleStyle.onNormal.textColor = IsDarkMode ? Color.black : Color.white;
            moduleToggleStyle.hover.textColor = IsDarkMode ? Color.white : Color.black;
            moduleToggleStyle.onHover.textColor = IsDarkMode ? Color.black : Color.white;
            moduleToggleStyle.active.textColor = IsDarkMode ? Color.white : Color.black;
            moduleToggleStyle.onActive.textColor = IsDarkMode ? Color.black : Color.white;
            moduleToggleStyle.alignment = TextAnchor.MiddleCenter;
            moduleToggleStyle.fontStyle = FontStyle.Bold;
            moduleToggleStyle.fixedWidth = 18f;
            moduleToggleStyle.fixedHeight = 18f;
            moduleToggleStyle.margin = new RectOffset(2, 2, 5, 5);

            closeButtonStyle = new GUIStyle(GUI.skin.button);
            closeButtonStyle.normal.background = closeTexture;
            closeButtonStyle.normal.textColor = AccentColor;
            closeButtonStyle.hover.background = closeTexture;
            closeButtonStyle.hover.textColor = AccentColor;
            closeButtonStyle.active.background = closeTexture;
            closeButtonStyle.active.textColor = AccentColor;
            closeButtonStyle.fontStyle = FontStyle.Bold;
            closeButtonStyle.alignment = TextAnchor.MiddleCenter;

            scrollViewBackgroundStyle = new GUIStyle(GUI.skin.box);
            scrollViewBackgroundStyle.normal.background = drawerBackgroundTexture;
            scrollViewBackgroundStyle.border = new RectOffset(0, 0, 0, 0);
            scrollViewBackgroundStyle.margin = new RectOffset(0, 0, 0, 0);
            scrollViewBackgroundStyle.padding = new RectOffset(0, 0, 0, 0);

            verticalScrollbarStyle = new GUIStyle(GUI.skin.verticalScrollbar);
            verticalScrollbarStyle.normal.background = scrollTrackTexture;
            verticalScrollbarStyle.hover.background = scrollTrackTexture;
            verticalScrollbarStyle.active.background = scrollTrackTexture;
            verticalScrollbarStyle.fixedWidth = 12f;

            verticalScrollbarThumbStyle = new GUIStyle(GUI.skin.verticalScrollbarThumb);
            verticalScrollbarThumbStyle.normal.background = scrollThumbTexture;
            verticalScrollbarThumbStyle.hover.background = scrollThumbTexture;
            verticalScrollbarThumbStyle.active.background = scrollThumbTexture;

            horizontalScrollbarStyle = new GUIStyle(GUI.skin.horizontalScrollbar);
            horizontalScrollbarStyle.normal.background = scrollTrackTexture;
            horizontalScrollbarStyle.hover.background = scrollTrackTexture;
            horizontalScrollbarStyle.active.background = scrollTrackTexture;
            horizontalScrollbarStyle.fixedHeight = 12f;

            horizontalScrollbarThumbStyle = new GUIStyle(GUI.skin.horizontalScrollbarThumb);
            horizontalScrollbarThumbStyle.normal.background = scrollThumbTexture;
            horizontalScrollbarThumbStyle.hover.background = scrollThumbTexture;
            horizontalScrollbarThumbStyle.active.background = scrollThumbTexture;

            horizontalSliderStyle = new GUIStyle(GUI.skin.horizontalSlider);
            horizontalSliderStyle.normal.background = sliderTrackTexture;
            horizontalSliderStyle.hover.background = sliderTrackTexture;
            horizontalSliderStyle.active.background = sliderTrackTexture;
            horizontalSliderStyle.fixedHeight = 10f;
            horizontalSliderStyle.border = new RectOffset(4, 4, 4, 4);

            horizontalSliderThumbStyle = new GUIStyle(GUI.skin.horizontalSliderThumb);
            horizontalSliderThumbStyle.normal.background = sliderThumbTexture;
            horizontalSliderThumbStyle.hover.background = sliderThumbTexture;
            horizontalSliderThumbStyle.active.background = sliderThumbTexture;
            horizontalSliderThumbStyle.fixedWidth = 12f;
            horizontalSliderThumbStyle.fixedHeight = 16f;

            GUI.skin.verticalScrollbarThumb = verticalScrollbarThumbStyle;
            GUI.skin.horizontalScrollbarThumb = horizontalScrollbarThumbStyle;
        }

        private void ResetStyles()
        {
            windowStyle = null;
            headerStyle = null;
            titleStyle = null;
            subtitleStyle = null;
            cardStyle = null;
            labelStyle = null;
            valueStyle = null;
            sectionTitleStyle = null;
            chipStyle = null;
            hintStyle = null;
            buttonStyle = null;
            drawerStyle = null;
            drawerItemStyle = null;
            drawerItemSelectedStyle = null;
            moduleToggleStyle = null;
            rowLabelStyle = null;
            rowValueStyle = null;
            closeButtonStyle = null;
            emptyStateStyle = null;
            verticalScrollbarStyle = null;
            verticalScrollbarThumbStyle = null;
            horizontalScrollbarStyle = null;
            horizontalScrollbarThumbStyle = null;
            scrollViewBackgroundStyle = null;
            horizontalSliderStyle = null;
            horizontalSliderThumbStyle = null;
            lastAppliedDarkMode = null;
        }

        private Texture2D MakeColorTexture(Color color)
        {
            Texture2D texture = new Texture2D(1, 1);
            texture.SetPixel(0, 0, color);
            texture.Apply();
            texture.hideFlags = HideFlags.HideAndDontSave;
            return texture;
        }

        private static Color WithAlpha(Color c, float alpha) =>
            new Color(c.r, c.g, c.b, Mathf.Clamp01(alpha));

        private float UIOpacity => StudioCharaEditor.UIOpacity != null
            ? Mathf.Clamp(StudioCharaEditor.UIOpacity.Value, 0.1f, 1f)
            : 0.88f;

        private void HandleWindowResize()
        {
            Event e = Event.current;
            Rect handleRect = new Rect(windowRect.width - resizeHitSize, windowRect.height - resizeHitSize, resizeHitSize, resizeHitSize);
            if (resizeControlId == 0)
            {
                resizeControlId = GUIUtility.GetControlID(windowID + 77, FocusType.Passive);
            }

            EventType controlEvent = e.GetTypeForControl(resizeControlId);
            switch (controlEvent)
            {
                case EventType.MouseDown:
                    if (e.button == 0 && handleRect.Contains(e.mousePosition))
                    {
                        GUIUtility.hotControl = resizeControlId;
                        GUIUtility.keyboardControl = 0;
                        isResizing = true;
                        resizeStartMousePos = e.mousePosition;
                        resizeStartSize = new Vector2(windowRect.width, windowRect.height);
                        e.Use();
                    }
                    break;
                case EventType.MouseDrag:
                    if (GUIUtility.hotControl == resizeControlId)
                    {
                        Vector2 dragDelta = e.mousePosition - resizeStartMousePos;
                        windowRect.width = Mathf.Max(minWindowWidth, resizeStartSize.x + dragDelta.x);
                        windowRect.height = Mathf.Max(minWindowHeight, resizeStartSize.y + dragDelta.y);
                        StudioCharaEditor.UIWidth.Value = (int)windowRect.width;
                        StudioCharaEditor.UIHeight.Value = (int)windowRect.height;
                        e.Use();
                    }
                    break;
                case EventType.MouseUp:
                    if (GUIUtility.hotControl == resizeControlId)
                    {
                        GUIUtility.hotControl = 0;
                        isResizing = false;
                        StudioCharaEditor.UIWidth.Value = (int)windowRect.width;
                        StudioCharaEditor.UIHeight.Value = (int)windowRect.height;
                        e.Use();
                    }
                    break;
            }
        }


        private void OnSelectChange(TreeNodeObject newSel)
        {
            lastSelectedTreeNode = newSel;
            CharaEditorMgr.Instance.UpdateSelection(newSel);
            SelectionContext selection = CharaEditorMgr.Instance.ActiveSelection;
            selectedTarget = selection.Target;
            ociTarget = selection.CharacterTarget;
            compatibleModuleNames.Clear();
            compatibleModuleNames.AddRange(selection.CompatibleModules.Select(module => module.DisplayName));
            //Console.WriteLine("Select change to {0}", ociTarget);
        }

    }
}