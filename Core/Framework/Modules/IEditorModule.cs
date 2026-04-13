using Studio;

namespace StudioModsMSG
{
    interface IEditorModule
    {
        string ModuleId { get; }
        string DisplayName { get; }
        int Priority { get; }
        bool CanHandle(ObjectCtrlInfo target);
    }
}
