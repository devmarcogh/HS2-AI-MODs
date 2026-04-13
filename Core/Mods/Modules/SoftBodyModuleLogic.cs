using BepInEx.Configuration;

namespace StudioModsMSG
{
    /// <summary>
    /// Logic/state management for SoftBody module (no GUI concerns)
    /// </summary>
    class SoftBodyModuleLogic
    {
        public bool IsEnabled
        {
            get => StudioCharaEditor.BreastSoftBodyEnabled.Value;
            set => StudioCharaEditor.BreastSoftBodyEnabled.Value = value;
        }

        public ConfigEntry<float> Intensity => StudioCharaEditor.BreastSoftBodyIntensity;
        public ConfigEntry<float> Stiffness => StudioCharaEditor.BreastSoftBodyStiffness;
        public ConfigEntry<float> Damping => StudioCharaEditor.BreastSoftBodyDamping;
        public ConfigEntry<float> MotionInfluence => StudioCharaEditor.BreastSoftBodyMotionInfluence;
        public ConfigEntry<float> MaxOffset => StudioCharaEditor.BreastSoftBodyMaxOffset;
        public ConfigEntry<float> WeightThreshold => StudioCharaEditor.BreastSoftBodyWeightThreshold;
        public ConfigEntry<float> WaveSpeed => StudioCharaEditor.BreastSoftBodyWaveSpeed;
        public ConfigEntry<float> LateralStrength => StudioCharaEditor.BreastSoftBodyLateralStrength;
        public ConfigEntry<float> SecondaryAmplitude => StudioCharaEditor.BreastSoftBodySecondaryAmplitude;
        public ConfigEntry<float> GravitySag => StudioCharaEditor.BreastSoftBodyGravitySag;

        public bool CanHandle(SelectionContext selection)
        {
            if (selection == null || selection.CharacterTarget == null || selection.CharacterTarget.charInfo == null)
            {
                return false;
            }
            return selection.CharacterTarget.charInfo.sex == 1;
        }

        public bool IsControllerAttached(SelectionContext selection)
        {
            if (selection == null || selection.CharacterTarget == null)
            {
                return false;
            }
            CharaEditorController controller = CharaEditorMgr.Instance.GetEditorController(selection.CharacterTarget);
            return controller != null && controller.HasBreastSoftBody;
        }

        public bool IsSoftBodyRuntimeActive(SelectionContext selection)
        {
            if (!CanHandle(selection) || !IsEnabled)
            {
                return false;
            }

            BreastSoftBodyRuntime runtime = selection.CharacterTarget.charInfo.gameObject.GetComponent<BreastSoftBodyRuntime>();
            return runtime != null && runtime.enabled;
        }
    }
}
