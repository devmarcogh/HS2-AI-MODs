using Studio;

namespace StudioModsMSG
{
    static class FeatureModuleIds
    {
        public const string SoftBody     = "mod.softbody";
        public const string ClothPhysics = "mod.clothphysics";
    }

    class SoftBodyModule : IEditorModule
    {
        public string ModuleId => FeatureModuleIds.SoftBody;
        public string DisplayName => "Breast SoftBody";
        public int Priority => 900;

        public bool CanHandle(ObjectCtrlInfo target)
        {
            OCIChar character = target as OCIChar;
            return character != null && character.charInfo != null && character.charInfo.sex == 1;
        }
    }

    class ClothPhysicsEditorModule : IEditorModule
    {
        public string ModuleId    => FeatureModuleIds.ClothPhysics;
        public string DisplayName => "Cloth Physics";
        public int Priority       => 880;

        public bool CanHandle(ObjectCtrlInfo target)
        {
            return target is OCIChar;
        }
    }
}
