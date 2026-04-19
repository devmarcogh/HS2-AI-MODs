using System;
using System.Collections.Generic;
using UnityEngine;

namespace StudioModsMSG
{
    /// <summary>
    /// Counting-sort spatial hash grid for fast nearest-neighbour queries.
    /// Zero GC allocations after first Build() at a given particle count.
    ///
    /// Uses the counting-sort pattern from SPH fluid solvers:
    ///   cellCount[hash] → how many particles in this cell
    ///   cellStart[hash] → start index into sorted entries array
    ///   entries[k]      → particle index (contiguous per cell)
    ///
    /// All particles sharing a cell are adjacent in memory, giving
    /// sequential cache-line reads during Query().  Two-pass build
    /// (count + scatter) is O(n) and cache-friendly.
    /// </summary>
    internal sealed class SpatialHashGrid
    {
        private float _cellSize = 0.05f;
        private float _invCell  = 20f;

        // Counting-sort arrays:
        //   _cellCount[hash] = number of particles in cell (used during build, then consumed)
        //   _cellStart[hash] = start index into _entries for this cell
        //   _entries[k]      = particle index (sorted by cell, contiguous)
        private int[] _cellCount;
        private int[] _cellStart;
        private int[] _entries;
        private int   _tableSize;
        private int   _count;

        // Tracks which cells were touched so we only clear those (not the whole table).
        private int[] _touchedCells;
        private int   _touchedCount;

        // Good primes for table sizes — keeps modulo distribution even.
        private static readonly int[] Primes =
        {
            251, 509, 1021, 2039, 4093, 8191, 16381, 32749, 65521
        };

        public bool IsBuilt => _count > 0;

        /// <summary>
        /// Inserts all positions into the grid via counting sort.
        /// Two passes over particles + one prefix-sum over touched cells.
        /// Zero GC after arrays are allocated to sufficient size.
        /// </summary>
        public void Build(Vector3[] positions, float cellSize)
        {
            _cellSize = cellSize;
            _invCell  = 1f / cellSize;

            int n = positions.Length;
            _count = n;

            // Size the hash table ≈ 2× particle count (prime)
            int desired = Math.Max(n * 2, 251);
            if (_cellCount == null || _tableSize < desired)
            {
                _tableSize = PickPrime(desired);
                _cellCount = new int[_tableSize];
                _cellStart = new int[_tableSize];
                _touchedCells = new int[_tableSize];
            }

            if (_entries == null || _entries.Length < n)
                _entries = new int[n];

            // Clear only cells that were used last frame
            for (int i = 0; i < _touchedCount; i++)
            {
                int c = _touchedCells[i];
                _cellCount[c] = 0;
                _cellStart[c] = 0;
            }
            _touchedCount = 0;

            // Pass 1: count particles per cell
            for (int i = 0; i < n; i++)
            {
                int hash = CellHash(positions[i]);
                if (_cellCount[hash] == 0)
                    _touchedCells[_touchedCount++] = hash;
                _cellCount[hash]++;
            }

            // Prefix sum: convert counts → start offsets
            // Only process touched cells (sparse iteration)
            for (int i = 0; i < _touchedCount; i++)
            {
                int c = _touchedCells[i];
                _cellStart[c] = 0; // will be assigned in sequential scan below
            }
            // Sequential prefix sum over the full touched set, sorted by cell index
            // to maintain stable ordering. Since we only need correct start offsets,
            // we accumulate in-place.
            {
                // Sort touched cells so prefix sum is deterministic
                Array.Sort(_touchedCells, 0, _touchedCount);
                int offset = 0;
                for (int i = 0; i < _touchedCount; i++)
                {
                    int c = _touchedCells[i];
                    _cellStart[c] = offset;
                    offset += _cellCount[c];
                }
            }

            // Pass 2: scatter particle indices into entries array
            // Use cellStart as write cursors (we'll restore them after)
            // We need a temp copy of cellStart — reuse cellCount as the cursor
            for (int i = 0; i < _touchedCount; i++)
            {
                int c = _touchedCells[i];
                _cellCount[c] = _cellStart[c]; // cursor = start
            }

            for (int i = 0; i < n; i++)
            {
                int hash = CellHash(positions[i]);
                _entries[_cellCount[hash]++] = i;
            }

            // Restore cellCount to actual counts (cursor - start = count)
            for (int i = 0; i < _touchedCount; i++)
            {
                int c = _touchedCells[i];
                _cellCount[c] -= _cellStart[c];
            }
        }

        /// <summary>
        /// Fills <paramref name="results"/> with indices of all particles within
        /// a cube of half-size <paramref name="radius"/> centred on <paramref name="pos"/>.
        /// The list is cleared before writing.
        /// Particles in each cell are contiguous in the entries array → sequential reads.
        /// </summary>
        public void Query(Vector3 pos, float radius, List<int> results)
        {
            results.Clear();
            if (_count == 0) return;

            int ri = (int)Math.Ceiling(radius * _invCell);
            int ox = (int)Math.Floor((double)(pos.x * _invCell));
            int oy = (int)Math.Floor((double)(pos.y * _invCell));
            int oz = (int)Math.Floor((double)(pos.z * _invCell));

            for (int dx = -ri; dx <= ri; dx++)
            for (int dy = -ri; dy <= ri; dy++)
            for (int dz = -ri; dz <= ri; dz++)
            {
                int hash = PackHash(ox + dx, oy + dy, oz + dz);
                int start = _cellStart[hash];
                int count = _cellCount[hash];
                for (int j = 0; j < count; j++)
                    results.Add(_entries[start + j]);
            }
        }

        /// <summary>Marks the grid as empty.</summary>
        public void Clear()
        {
            _count = 0;
        }

        // ── Hashing helpers ──────────────────────────────────────────────────

        private int CellHash(Vector3 p)
        {
            int ix = (int)Math.Floor((double)(p.x * _invCell));
            int iy = (int)Math.Floor((double)(p.y * _invCell));
            int iz = (int)Math.Floor((double)(p.z * _invCell));
            return PackHash(ix, iy, iz);
        }

        private int PackHash(int x, int y, int z)
        {
            unchecked
            {
                int h = x * 73856093 ^ y * 19349663 ^ z * 83492791;
                return ((h % _tableSize) + _tableSize) % _tableSize;
            }
        }

        private static int PickPrime(int minSize)
        {
            for (int i = 0; i < Primes.Length; i++)
                if (Primes[i] >= minSize) return Primes[i];
            return Primes[Primes.Length - 1];
        }
    }
}
