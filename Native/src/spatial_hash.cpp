#include "studio_mods_native.h"
#include <cmath>
#include <cstring>
#include <algorithm>
#include <immintrin.h>

// ─── Primes for table sizing ────────────────────────────────────
static const int kPrimes[] = {
    251, 509, 1021, 2039, 4093, 8191, 16381, 32749, 65521
};

static int PickPrime(int desired) {
    for (int p : kPrimes)
        if (p >= desired) return p;
    return kPrimes[sizeof(kPrimes) / sizeof(kPrimes[0]) - 1];
}

// ─── Hash function (same as C# SpatialHashGrid) ────────────────
static inline int HashCell(int ix, int iy, int iz, int tableSize) {
    // Large primes for spatial hashing
    unsigned h = (unsigned)(ix * 73856093) ^ (unsigned)(iy * 19349663) ^ (unsigned)(iz * 83492791);
    return (int)(h % (unsigned)tableSize);
}

// ─── Context ────────────────────────────────────────────────────
struct SHashContext {
    float  cellSize;
    float  invCell;
    int    tableSize;
    int    count;

    int*   cellCount;   // [tableSize]
    int*   cellStart;   // [tableSize]
    int*   entries;     // [count] sorted particle indices
    int*   hashes;      // [count] per-particle hash (avoid recompute in scatter)

    int*   touchedCells;
    int    touchedCount;

    int    allocCap;    // allocated capacity for entries/hashes
    int    allocTable;  // allocated table size

    const float* cachedPositions; // pointer to last Build() positions for Query distance filtering
};

static void EnsureCapacity(SHashContext* ctx, int count, int tableSize) {
    if (count > ctx->allocCap) {
        delete[] ctx->entries;
        delete[] ctx->hashes;
        ctx->entries  = new int[count];
        ctx->hashes   = new int[count];
        ctx->allocCap = count;
    }
    if (tableSize > ctx->allocTable) {
        delete[] ctx->cellCount;
        delete[] ctx->cellStart;
        delete[] ctx->touchedCells;
        ctx->cellCount   = new int[tableSize]();
        ctx->cellStart   = new int[tableSize]();
        ctx->touchedCells = new int[tableSize];
        ctx->allocTable  = tableSize;
    }
    ctx->tableSize = tableSize;
}

// ─── Create / Destroy ───────────────────────────────────────────
SMODS_API int64_t SHash_Create(int capacity) {
    auto* ctx = new SHashContext();
    std::memset(ctx, 0, sizeof(SHashContext));

    int ts = PickPrime(std::max(capacity * 2, 251));
    EnsureCapacity(ctx, capacity, ts);
    return reinterpret_cast<int64_t>(ctx);
}

SMODS_API void SHash_Destroy(int64_t handle) {
    auto* ctx = reinterpret_cast<SHashContext*>(handle);
    if (!ctx) return;
    delete[] ctx->cellCount;
    delete[] ctx->cellStart;
    delete[] ctx->entries;
    delete[] ctx->hashes;
    delete[] ctx->touchedCells;
    delete ctx;
}

// ─── Build ──────────────────────────────────────────────────────
SMODS_API void SHash_Build(int64_t handle, const float* positions, int count, float cellSize) {
    auto* ctx = reinterpret_cast<SHashContext*>(handle);
    if (!ctx || count == 0) return;

    ctx->cellSize = cellSize;
    ctx->invCell  = 1.0f / cellSize;
    ctx->count    = count;

    int ts = PickPrime(std::max(count * 2, 251));
    EnsureCapacity(ctx, count, ts);

    // Clear only touched cells from last build
    for (int i = 0; i < ctx->touchedCount; i++) {
        int c = ctx->touchedCells[i];
        ctx->cellCount[c] = 0;
        ctx->cellStart[c] = 0;
    }
    ctx->touchedCount = 0;

    float inv = ctx->invCell;

    // Pass 1: count particles per cell + compute hashes
    for (int i = 0; i < count; i++) {
        int ix = (int)std::floor(positions[i * 3]     * inv);
        int iy = (int)std::floor(positions[i * 3 + 1] * inv);
        int iz = (int)std::floor(positions[i * 3 + 2] * inv);
        int h = HashCell(ix, iy, iz, ts);
        ctx->hashes[i] = h;

        if (ctx->cellCount[h] == 0) {
            ctx->touchedCells[ctx->touchedCount++] = h;
        }
        ctx->cellCount[h]++;
    }

    // Prefix sum → cellStart
    int running = 0;
    for (int i = 0; i < ctx->touchedCount; i++) {
        int c = ctx->touchedCells[i];
        ctx->cellStart[c] = running;
        running += ctx->cellCount[c];
        ctx->cellCount[c] = 0; // reset for scatter pass
    }

    // Pass 2: scatter into sorted order
    for (int i = 0; i < count; i++) {
        int h = ctx->hashes[i];
        int slot = ctx->cellStart[h] + ctx->cellCount[h];
        ctx->entries[slot] = i;
        ctx->cellCount[h]++;
    }

    // Cache position pointer for distance filtering in Query
    ctx->cachedPositions = positions;
}

// ─── Query ──────────────────────────────────────────────────────
SMODS_API int SHash_Query(
    int64_t handle,
    float cx, float cy, float cz,
    float radius,
    int* outIndices,
    int maxResults)
{
    auto* ctx = reinterpret_cast<SHashContext*>(handle);
    if (!ctx || ctx->count == 0) return 0;

    float inv = ctx->invCell;
    float r2 = radius * radius;
    int ts = ctx->tableSize;
    int found = 0;

    // Determine cell range to check
    int minIx = (int)std::floor((cx - radius) * inv);
    int maxIx = (int)std::floor((cx + radius) * inv);
    int minIy = (int)std::floor((cy - radius) * inv);
    int maxIy = (int)std::floor((cy + radius) * inv);
    int minIz = (int)std::floor((cz - radius) * inv);
    int maxIz = (int)std::floor((cz + radius) * inv);

    const float* positions = ctx->cachedPositions;

    for (int ix = minIx; ix <= maxIx; ix++) {
        for (int iy = minIy; iy <= maxIy; iy++) {
            for (int iz = minIz; iz <= maxIz; iz++) {
                int h = HashCell(ix, iy, iz, ts);
                int start = ctx->cellStart[h];
                int count = ctx->cellCount[h];

                for (int k = 0; k < count && found < maxResults; k++) {
                    int pidx = ctx->entries[start + k];
                    // Distance check to eliminate false positives from hash cell corners
                    if (positions) {
                        float dx = positions[pidx * 3]     - cx;
                        float dy = positions[pidx * 3 + 1] - cy;
                        float dz = positions[pidx * 3 + 2] - cz;
                        if (dx * dx + dy * dy + dz * dz > r2) continue;
                    }
                    outIndices[found++] = pidx;
                }
            }
        }
    }

    return found;
}
