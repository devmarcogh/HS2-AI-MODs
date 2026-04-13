using System.Collections.Generic;
using AIChara;
using Studio;

namespace StudioModsMSG
{
    class SelectionContext
    {
        public TreeNodeObject Node { get; private set; }
        public ObjectCtrlInfo Target { get; private set; }
        public OCIChar CharacterTarget { get; private set; }
        public List<IEditorModule> CompatibleModules { get; private set; }

        public bool HasSelection => Target != null;
        public bool IsCharacter => CharacterTarget != null;

        public string TargetTypeName
        {
            get
            {
                if (Target == null)
                {
                    return "None";
                }

                return Target.GetType().Name;
            }
        }

        public static SelectionContext Empty { get; } = new SelectionContext(null, null, null, new List<IEditorModule>());

        public SelectionContext(TreeNodeObject node, ObjectCtrlInfo target, OCIChar characterTarget, List<IEditorModule> compatibleModules)
        {
            Node = node;
            Target = target;
            CharacterTarget = characterTarget;
            CompatibleModules = compatibleModules ?? new List<IEditorModule>();
        }
    }
}
