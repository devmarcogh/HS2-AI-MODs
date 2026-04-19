using System.Collections.Generic;
using Studio;
using UnityEngine;

namespace StudioModsMSG
{
    /// <summary>
    /// Watches Studio's scene item dictionary for Joan6694 Dynamic Bone Collider items
    /// (GroupNo=11 / CategoryNo=42, GUID="com.joan6694.dynamicBoneColliders") and
    /// maintains a flat list of their DynamicBoneColliderBase components so that
    /// ClothSoftBodyRuntime can treat them as additional scene colliders.
    ///
    /// Call <see cref="Tick"/> every N frames (not every frame — dicInfo iteration is O(n)).
    /// </summary>
    static class Joan6694ColliderWatcher
    {
        // ── Joan6694 Studio info identifiers ─────────────────────────────────
        // GroupNo=11, CategoryNo=42 as seen in QuickAccessBox debug logs.
        private const int Joan6694Group    = 11;
        private const int Joan6694Category = 42;

        // ── Tracked colliders ─────────────────────────────────────────────────
        private static readonly List<DynamicBoneColliderBase> _colliders =
            new List<DynamicBoneColliderBase>(8);

        /// <summary>
        /// All Joan6694 DynamicBoneCollider components currently in the scene.
        /// This list is rebuilt every time <see cref="Tick"/> is called.
        /// </summary>
        public static IReadOnlyList<DynamicBoneColliderBase> Colliders => _colliders;

        // ── Frame-based throttle ──────────────────────────────────────────────
        private static int _lastScanFrame = -999;
        private const  int ScanInterval   = 30; // re-scan every 30 frames

        /// <summary>
        /// Should be called from CharaEditorMgr.HouseKeeping (or similar Update path).
        /// Internally throttles to once every <see cref="ScanInterval"/> frames.
        /// </summary>
        public static void Tick()
        {
            int frame = Time.frameCount;
            if (frame - _lastScanFrame < ScanInterval) return;
            _lastScanFrame = frame;
            RebuildList();
        }

        private static void RebuildList()
        {
            _colliders.Clear();

            var studio = Studio.Studio.Instance;
            if (studio == null) return;

            var dicInfo = studio.dicInfo;
            if (dicInfo == null) return;

            foreach (var kv in dicInfo)
            {
                OCIItem item = kv.Value as OCIItem;
                if (item == null) continue;

                // Match by GroupNo + CategoryNo (fastest check, avoids string GUID lookup)
                Studio.OIItemInfo itemInfo = item.itemInfo;
                if (itemInfo == null) continue;
                if (itemInfo.group != Joan6694Group || itemInfo.category != Joan6694Category) continue;

                // Gather all DynamicBoneColliderBase on the item's GameObject hierarchy
                Transform root = item.objectItem?.transform;
                if (root == null) continue;

                DynamicBoneColliderBase[] found = root.GetComponentsInChildren<DynamicBoneColliderBase>(true);
                for (int i = 0; i < found.Length; i++)
                {
                    if (found[i] != null && !_colliders.Contains(found[i]))
                        _colliders.Add(found[i]);
                }
            }
        }

        /// <summary>Force an immediate rescan (e.g. when a scene is loaded).</summary>
        public static void ForceRescan()
        {
            _lastScanFrame = -999;
            RebuildList();
        }
    }
}
