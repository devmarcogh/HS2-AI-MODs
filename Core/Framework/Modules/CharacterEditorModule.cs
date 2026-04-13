using Studio;

namespace StudioModsMSG
{
    class CharacterEditorModule : IEditorModule
    {
        public string ModuleId => "core.character";
        public string DisplayName => "Character Editor Core";
        public int Priority => 1000;

        public bool CanHandle(ObjectCtrlInfo target)
        {
            return target is OCIChar;
        }
    }
}
