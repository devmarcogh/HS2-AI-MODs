using AIChara;
using System.Collections.Generic;
using UnityEngine;

namespace StudioModsMSG
{
    /// <summary>
    /// Logic layer for the Cloth Physics module.
    /// Mirrors the pattern of SoftBodyModuleLogic: no GUI concerns here.
    /// </summary>
    class ClothPhysicsModule
    {
        public bool CanHandle(SelectionContext selection)
        {
            return selection != null
                && selection.CharacterTarget != null
                && selection.CharacterTarget.charInfo != null;
        }

        /// <summary>
        /// Returns the ClothSoftBodyRuntime attached to the character,
        /// creating and attaching one if it doesn't exist yet.
        /// </summary>
        public ClothSoftBodyRuntime GetOrCreateRuntime(SelectionContext selection)
        {
            if (!CanHandle(selection)) return null;
            ChaControl chaCtrl = selection.CharacterTarget.charInfo;
            ClothSoftBodyRuntime rt = chaCtrl.gameObject.GetComponent<ClothSoftBodyRuntime>();
            if (rt == null)
            {
                rt = chaCtrl.gameObject.AddComponent<ClothSoftBodyRuntime>();
                rt.Attach(chaCtrl);
            }
            return rt;
        }

        /// <summary>
        /// Returns the cloth entry list for the character (may build it lazily).
        /// </summary>
        public IReadOnlyList<ClothPhysicsEntry> GetEntries(SelectionContext selection)
        {
            ClothSoftBodyRuntime rt = GetOrCreateRuntime(selection);
            return rt != null ? rt.Entries : null;
        }

        public void ActivateMesh(SelectionContext selection, ClothMeshState state)
        {
            ClothSoftBodyRuntime rt = GetOrCreateRuntime(selection);
            rt?.ActivateMesh(state);
        }

        public void DeactivateMesh(SelectionContext selection, ClothMeshState state)
        {
            ClothSoftBodyRuntime rt = GetOrCreateRuntime(selection);
            rt?.DeactivateMesh(state);
        }

        public void ToggleMesh(SelectionContext selection, ClothMeshState state)
        {
            if (state.IsActive) DeactivateMesh(selection, state);
            else                ActivateMesh(selection, state);
        }

        public void RefreshEntries(SelectionContext selection)
        {
            ClothSoftBodyRuntime rt = GetOrCreateRuntime(selection);
            rt?.RefreshEntries();
        }

        public void RecomputePins(SelectionContext selection, ClothMeshState state)
        {
            ClothSoftBodyRuntime rt = GetOrCreateRuntime(selection);
            rt?.RecomputePinsFromSelectedBone(state);
        }

        /// <summary>
        /// Builds auto-capsule and proxy-particle colliders for the character
        /// based on the per-group bone modes chosen in the GUI.
        /// </summary>
        public void RebuildColliders(SelectionContext selection, Dictionary<string, ColliderMode> boneGroupModes)
        {
            ClothSoftBodyRuntime rt = GetOrCreateRuntime(selection);
            rt?.BuildAutoColliders(boneGroupModes);
        }
    }
}
