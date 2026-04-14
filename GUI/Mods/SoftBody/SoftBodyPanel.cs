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

        private sealed class SoftBodyConfigSnapshot
        {
            public float Intensity;
            public float Stiffness;
            public float Damping;
            public float MotionInfluence;
            public float MaxOffset;
            public float WeightThreshold;
            public float WaveSpeed;
            public float LateralStrength;
            public float SecondaryAmplitude;
            public float GravitySag;
        }

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

        public bool TryCopyConfig(SelectionContext selection, out object config)
        {
            config = new SoftBodyConfigSnapshot
            {
                Intensity = logic.Intensity.Value,
                Stiffness = logic.Stiffness.Value,
                Damping = logic.Damping.Value,
                MotionInfluence = logic.MotionInfluence.Value,
                MaxOffset = logic.MaxOffset.Value,
                WeightThreshold = logic.WeightThreshold.Value,
                WaveSpeed = logic.WaveSpeed.Value,
                LateralStrength = logic.LateralStrength.Value,
                SecondaryAmplitude = logic.SecondaryAmplitude.Value,
                GravitySag = logic.GravitySag.Value,
            };
            return true;
        }

        public bool TryPasteConfig(SelectionContext selection, object config)
        {
            SoftBodyConfigSnapshot snap = config as SoftBodyConfigSnapshot;
            if (snap == null) return false;

            logic.Intensity.Value = snap.Intensity;
            logic.Stiffness.Value = snap.Stiffness;
            logic.Damping.Value = snap.Damping;
            logic.MotionInfluence.Value = snap.MotionInfluence;
            logic.MaxOffset.Value = snap.MaxOffset;
            logic.WeightThreshold.Value = snap.WeightThreshold;
            logic.WaveSpeed.Value = snap.WaveSpeed;
            logic.LateralStrength.Value = snap.LateralStrength;
            logic.SecondaryAmplitude.Value = snap.SecondaryAmplitude;
            logic.GravitySag.Value = snap.GravitySag;
            return true;
        }
    }
}
