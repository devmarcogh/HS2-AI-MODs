using System;
using System.Collections.Generic;
using System.Linq;

namespace StudioModsMSG
{
    class ModuleRegistry
    {
        private readonly Dictionary<string, IEditorModule> modulesById = new Dictionary<string, IEditorModule>(StringComparer.OrdinalIgnoreCase);

        public IEnumerable<IEditorModule> Modules => modulesById.Values;

        public bool Register(IEditorModule module)
        {
            if (module == null || string.IsNullOrWhiteSpace(module.ModuleId))
            {
                return false;
            }

            if (modulesById.ContainsKey(module.ModuleId))
            {
                return false;
            }

            modulesById[module.ModuleId] = module;
            return true;
        }

        public List<IEditorModule> GetCompatibleModules(Studio.ObjectCtrlInfo target)
        {
            return modulesById.Values
                .Where(module => module.CanHandle(target))
                .OrderByDescending(module => module.Priority)
                .ThenBy(module => module.DisplayName)
                .ToList();
        }
    }
}
