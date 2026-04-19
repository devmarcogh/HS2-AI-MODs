using System;
using UnityEngine;

namespace StudioModsMSG
{
    /// <summary>
    /// Factory for creating module panel instances
    /// </summary>
    static class ModulePanelFactory
    {
        public static IModulePanelUI[] CreateDefaultPanels()
        {
            return new IModulePanelUI[]
            {
                new ClothPhysicsPanel(),
            };
        }
    }
}
