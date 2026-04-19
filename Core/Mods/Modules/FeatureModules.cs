using Studio;

namespace StudioModsMSG
{
    static class FeatureModuleIds
    {
        public const string ClothPhysics   = "mod.clothphysics";
        public const string ClothCollider  = "mod.clothcollider";
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
