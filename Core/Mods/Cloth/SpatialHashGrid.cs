using System;
using System.Collections.Generic;
using UnityEngine;

namespace StudioModsMSG
{
    /// <summary>
    /// Uniform-cell spatial hash grid for fast nearest-neighbour queries.
    ///
    /// Usage pattern (zero-GC in hot path):
    ///   1. Call Build() once per frame after positions are updated.
    ///   2. Call Query() for each cloth vertex; pass a pre-allocated List&lt;int&gt;.
    ///
    /// Implementation notes:
    ///   – Cell coordinate range: ±8192 cells per axis → ±409 m with 5 cm cells.
    ///   – Existing per-cell List&lt;int&gt; objects are reused across frames to minimise GC.
    ///     New cells allocate a small List only on first encounter.
    /// </summary>
    internal sealed class SpatialHashGrid
    {
        private float _cellSize   = 0.05f;
        private float _invCell    = 20f;

        // Per-cell particle index lists.  Cleared each Build() but the List objects are kept.
        private readonly Dictionary<long, List<int>> _cells = new Dictionary<long, List<int>>();

        public bool IsBuilt => _cells.Count > 0;

        /// <summary>
        /// Inserts all positions into the grid.
        /// </summary>
        public void Build(Vector3[] positions, float cellSize)
        {
            _cellSize = cellSize;
            _invCell  = 1f / cellSize;

            // Keep allocated List<int> objects but clear their contents
            foreach (var kv in _cells) kv.Value.Clear();

            for (int i = 0; i < positions.Length; i++)
            {
                long key = PosToKey(positions[i]);
                List<int> cell;
                if (!_cells.TryGetValue(key, out cell))
                    _cells[key] = cell = new List<int>(4);
                cell.Add(i);
            }
        }

        /// <summary>
        /// Fills <paramref name="results"/> with indices of all particles within
        /// a cube of half-size <paramref name="radius"/> centred on <paramref name="pos"/>.
        /// The list is cleared before writing.
        /// </summary>
        public void Query(Vector3 pos, float radius, List<int> results)
        {
            results.Clear();
            if (_cells.Count == 0) return;

            int ri = (int)Math.Ceiling(radius * _invCell);
            int ox = (int)Math.Floor((double)(pos.x * _invCell));
            int oy = (int)Math.Floor((double)(pos.y * _invCell));
            int oz = (int)Math.Floor((double)(pos.z * _invCell));

            for (int dx = -ri; dx <= ri; dx++)
            for (int dy = -ri; dy <= ri; dy++)
            for (int dz = -ri; dz <= ri; dz++)
            {
                List<int> cell;
                if (_cells.TryGetValue(PackKey(ox + dx, oy + dy, oz + dz), out cell))
                    for (int j = 0; j < cell.Count; j++) results.Add(cell[j]);
            }
        }

        /// <summary>Removes all entries without releasing cell list objects.</summary>
        public void Clear()
        {
            foreach (var kv in _cells) kv.Value.Clear();
        }

        // ── Key helpers ──────────────────────────────────────────────────────

        private long PosToKey(Vector3 p)
            => PackKey(
                (int)Math.Floor((double)(p.x * _invCell)),
                (int)Math.Floor((double)(p.y * _invCell)),
                (int)Math.Floor((double)(p.z * _invCell)));

        // Pack three signed ints into a long (14 bits each, offset by 8192 to handle negatives).
        // Covers ±8191 cells on each axis → ±400 m with 5 cm cells.
        private static long PackKey(int x, int y, int z)
        {
            const int OFF  = 8192;
            const int BITS = 14;
            const int MASK = (1 << BITS) - 1;   // 0x3FFF

            long kx = (long)((x + OFF) & MASK);
            long ky = (long)((y + OFF) & MASK);
            long kz = (long)((z + OFF) & MASK);
            return kx | (ky << BITS) | (kz << (BITS * 2));
        }
    }
}
