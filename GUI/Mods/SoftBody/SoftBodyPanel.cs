using System;
using UnityEngine;

namespace StudioModsMSG
{
    /// <summary>
    /// UI Panel for SoftBody module (GUI only, no logic)
    /// </summary>
    class SoftBodyPanel : IModulePanelUI
    {
        private readonly SoftBodyModuleLogic logic = new SoftBodyModuleLogic();

        public string ModuleId => FeatureModuleIds.SoftBody;

        public bool GetDefaultEnabledState()
        {
            return logic.IsEnabled;
        }

        public void SyncEnabledState(ref bool enabled)
        {
            enabled = logic.IsEnabled;
        }

        public void OnToggleChanged(bool enabled)
        {
            logic.IsEnabled = enabled;
        }

        public void Draw(SelectionContext selection, BaseUI host)
        {
            if (!logic.CanHandle(selection))
            {
                GUILayout.Label("SoftBody only applies to female characters in HS2.", host.HintStyleRef);
                return;
            }

            bool hasRuntime = logic.IsSoftBodyRuntimeActive(selection);
            host.DrawStatRow("Runtime Status", hasRuntime ? "Active" : "Inactive", 
                hasRuntime ? host.SuccessColorRef : host.WarningColorRef);
            
            if (!hasRuntime)
            {
                GUILayout.Space(6f);
                GUILayout.Label("Enable the toggle in the drawer on the left to activate SoftBody simulation.", host.HintStyleRef);
                return;
            }

            GUILayout.Space(8f);
            host.DrawFloatSliderSetting("Intensity", logic.Intensity, 0f, 10f);
            host.DrawFloatSliderSetting("Stiffness", logic.Stiffness, 1f, 800f);
            host.DrawFloatSliderSetting("Damping", logic.Damping, 0.5f, 250f);
            host.DrawFloatSliderSetting("Motion Influence", logic.MotionInfluence, 0f, 20f);
            host.DrawFloatSliderSetting("Max Offset", logic.MaxOffset, 0.001f, 0.8f);
            host.DrawFloatSliderSetting("Weight Threshold", logic.WeightThreshold, 0.005f, 5f);
            host.DrawFloatSliderSetting("Wave Ripple", logic.WaveSpeed, 0f, 15f);
            host.DrawFloatSliderSetting("Lateral Sway", logic.LateralStrength, 0f, 2f);
            host.DrawFloatSliderSetting("Secondary Wobble", logic.SecondaryAmplitude, 0f, 1f);
            host.DrawFloatSliderSetting("Gravity Sag", logic.GravitySag, 0f, 0.05f);
            GUILayout.Space(6f);
            GUILayout.Label("SoftBody physics simulates natural breast movement. Adjust sliders to fine-tune the effect.", host.HintStyleRef);
        }
    }
}
