using System;
using BepInEx.Configuration;
using UnityEngine;

namespace StudioModsMSG
{
    interface IModulePanelUI
    {
        string ModuleId { get; }
        bool GetDefaultEnabledState();
        void SyncEnabledState(ref bool enabled);
        void OnToggleChanged(bool enabled);
        void Draw(SelectionContext selection, BaseUI host);
        bool TryCopyConfig(SelectionContext selection, out object config);
        bool TryPasteConfig(SelectionContext selection, object config);
    }

    class InfoModulePanel : IModulePanelUI
    {
        private readonly string moduleId;
        private readonly Func<CharaEditorController, bool> runtimeCheck;
        private readonly string readyText;
        private readonly string missingText;

        public string ModuleId => moduleId;

        public InfoModulePanel(string moduleId, Func<CharaEditorController, bool> runtimeCheck, string readyText, string missingText)
        {
            this.moduleId = moduleId;
            this.runtimeCheck = runtimeCheck;
            this.readyText = readyText;
            this.missingText = missingText;
        }

        public bool GetDefaultEnabledState()
        {
            return true;
        }

        public void SyncEnabledState(ref bool enabled)
        {
        }

        public void OnToggleChanged(bool enabled)
        {
        }

        public void Draw(SelectionContext selection, BaseUI host)
        {
            CharaEditorController controller = CharaEditorMgr.Instance.GetEditorController(selection.CharacterTarget);
            bool available = runtimeCheck == null || runtimeCheck(controller);
            host.DrawStatRow("Runtime", available ? "Detected" : "Not detected", available ? host.SuccessColorRef : host.WarningColorRef);
            GUILayout.Space(8f);
            GUILayout.Label(available ? readyText : missingText, host.HintStyleRef);
        }

        public bool TryCopyConfig(SelectionContext selection, out object config)
        {
            config = null;
            return false;
        }

        public bool TryPasteConfig(SelectionContext selection, object config)
        {
            return false;
        }
    }


}
