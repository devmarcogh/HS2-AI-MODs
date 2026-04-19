#include "studio_mods_native.h"
#include <cmath>
#include <cstring>
#include <algorithm>
#include <immintrin.h>

// ─── Helpers ────────────────────────────────────────────────────
static inline float fast_rsqrt_sdf(float x) {
    __m128 v = _mm_set_ss(x);
    __m128 r = _mm_rsqrt_ss(v);
    // One Newton-Raphson iteration for ~22-bit precision
    __m128 half  = _mm_set_ss(0.5f);
    __m128 three = _mm_set_ss(3.0f);
    __m128 muls  = _mm_mul_ss(_mm_mul_ss(v, r), r);
    r = _mm_mul_ss(_mm_mul_ss(r, _mm_sub_ss(three, muls)), half);
    return _mm_cvtss_f32(r);
}

static inline unsigned int SDF_HashCell(int ix, int iy, int iz, int tableSize) {
    unsigned h = (unsigned)(ix * 73856093) ^ (unsigned)(iy * 19349663) ^ (unsigned)(iz * 83492791);
    return h % (unsigned)tableSize;
}

// ─── SDF Context ────────────────────────────────────────────────
struct SDFContext {
    // Grid params
    int   resX, resY, resZ;
    int   totalVoxels;
    float originX, originY, originZ;
    float cellSize;
    float invCellSize;
    float maxDist;

    // SDF data (owned)
    float* sdfData;
    int    sdfCapacity;

    // Spatial hash for vertex lookup (simple embedded version)
    int    hashTableSize;
    int*   hashCellCount;
    int*   hashCellStart;
    int*   hashEntries;
    int*   hashPerParticle; // per-particle hash
    int    hashCapacity;
    int    hashTableCapacity;
};

// ─── Primes ─────────────────────────────────────────────────────
static const int kSDFPrimes[] = { 251, 509, 1021, 2039, 4093, 8191, 16381, 32749, 65521 };

static int SDFPickPrime(int desired) {
    for (int p : kSDFPrimes)
        if (p >= desired) return p;
    return kSDFPrimes[sizeof(kSDFPrimes) / sizeof(kSDFPrimes[0]) - 1];
}

SMODS_API int64_t SDF_Create(int maxVoxels, int maxVertices) {
    auto* ctx = new SDFContext();
    std::memset(ctx, 0, sizeof(SDFContext));

    ctx->sdfData = new float[maxVoxels];
    ctx->sdfCapacity = maxVoxels;

    int ts = SDFPickPrime(std::max(maxVertices * 2, 251));
    ctx->hashTableSize = ts;
    ctx->hashTableCapacity = ts;
    ctx->hashCellCount = new int[ts]();
    ctx->hashCellStart = new int[ts]();
    ctx->hashEntries = new int[maxVertices];
    ctx->hashPerParticle = new int[maxVertices];
    ctx->hashCapacity = maxVertices;

    return reinterpret_cast<int64_t>(ctx);
}

SMODS_API void SDF_Destroy(int64_t handle) {
    auto* ctx = reinterpret_cast<SDFContext*>(handle);
    if (!ctx) return;
    delete[] ctx->sdfData;
    delete[] ctx->hashCellCount;
    delete[] ctx->hashCellStart;
    delete[] ctx->hashEntries;
    delete[] ctx->hashPerParticle;
    delete ctx;
}

// ─── Skin vertices (position + normal) ─────────────────────────
SMODS_API void SDF_SkinVertices(
    const float* bindPos,      // [count*3]
    const float* bindNormal,   // [count*3]
    const int*   boneIdx,      // [count*4]
    const float* boneW,        // [count*4]
    const float* boneMats,     // [boneCount*16] column-major
    int          count,
    float*       outPos,       // [count*3]
    float*       outNormal)    // [count*3]
{
    for (int i = 0; i < count; i++) {
        float px = bindPos[i * 3], py = bindPos[i * 3 + 1], pz = bindPos[i * 3 + 2];
        float nx = bindNormal[i * 3], ny = bindNormal[i * 3 + 1], nz = bindNormal[i * 3 + 2];

        float rpx = 0, rpy = 0, rpz = 0;
        float rnx = 0, rny = 0, rnz = 0;

        for (int b = 0; b < 4; b++) {
            float w = boneW[i * 4 + b];
            if (w <= 0.0f) continue;
            int bi = boneIdx[i * 4 + b];
            const float* m = &boneMats[bi * 16];

            // Position: M * [px,py,pz,1]
            rpx += w * (m[0] * px + m[4] * py + m[8]  * pz + m[12]);
            rpy += w * (m[1] * px + m[5] * py + m[9]  * pz + m[13]);
            rpz += w * (m[2] * px + m[6] * py + m[10] * pz + m[14]);

            // Normal: M * [nx,ny,nz,0] (direction, no translation)
            rnx += w * (m[0] * nx + m[4] * ny + m[8]  * nz);
            rny += w * (m[1] * nx + m[5] * ny + m[9]  * nz);
            rnz += w * (m[2] * nx + m[6] * ny + m[10] * nz);
        }

        outPos[i * 3] = rpx; outPos[i * 3 + 1] = rpy; outPos[i * 3 + 2] = rpz;

        // Normalize normal
        float nlen = std::sqrt(rnx * rnx + rny * rny + rnz * rnz);
        if (nlen > 1e-6f) {
            float inv = 1.0f / nlen;
            outNormal[i * 3] = rnx * inv;
            outNormal[i * 3 + 1] = rny * inv;
            outNormal[i * 3 + 2] = rnz * inv;
        } else {
            outNormal[i * 3] = 0; outNormal[i * 3 + 1] = 1; outNormal[i * 3 + 2] = 0;
        }
    }
}

// ─── Point-Triangle distance helper ────────────────────────────
// Returns the squared distance from point p to triangle (a, b, c).
// Also outputs the closest point on the triangle in 'closest'.
static float PointTriangleDistSq(
    float px, float py, float pz,
    float ax, float ay, float az,
    float bx, float by, float bz,
    float cx, float cy, float cz,
    float& closestX, float& closestY, float& closestZ)
{
    float abx = bx - ax, aby = by - ay, abz = bz - az;
    float acx = cx - ax, acy = cy - ay, acz = cz - az;
    float apx = px - ax, apy = py - ay, apz = pz - az;

    float d1 = abx * apx + aby * apy + abz * apz;
    float d2 = acx * apx + acy * apy + acz * apz;
    if (d1 <= 0.0f && d2 <= 0.0f) {
        closestX = ax; closestY = ay; closestZ = az;
        return apx * apx + apy * apy + apz * apz;
    }

    float bpx = px - bx, bpy = py - by, bpz = pz - bz;
    float d3 = abx * bpx + aby * bpy + abz * bpz;
    float d4 = acx * bpx + acy * bpy + acz * bpz;
    if (d3 >= 0.0f && d4 <= d3) {
        closestX = bx; closestY = by; closestZ = bz;
        return bpx * bpx + bpy * bpy + bpz * bpz;
    }

    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
        float v = d1 / (d1 - d3);
        closestX = ax + abx * v; closestY = ay + aby * v; closestZ = az + abz * v;
        float dx = px - closestX, dy = py - closestY, dz = pz - closestZ;
        return dx * dx + dy * dy + dz * dz;
    }

    float cpx = px - cx, cpy = py - cy, cpz = pz - cz;
    float d5 = abx * cpx + aby * cpy + abz * cpz;
    float d6 = acx * cpx + acy * cpy + acz * cpz;
    if (d6 >= 0.0f && d5 <= d6) {
        closestX = cx; closestY = cy; closestZ = cz;
        return cpx * cpx + cpy * cpy + cpz * cpz;
    }

    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
        float w = d2 / (d2 - d6);
        closestX = ax + acx * w; closestY = ay + acy * w; closestZ = az + acz * w;
        float dx = px - closestX, dy = py - closestY, dz = pz - closestZ;
        return dx * dx + dy * dy + dz * dz;
    }

    float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
        float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        closestX = bx + (cx - bx) * w; closestY = by + (cy - by) * w; closestZ = bz + (cz - bz) * w;
        float dx = px - closestX, dy = py - closestY, dz = pz - closestZ;
        return dx * dx + dy * dy + dz * dz;
    }

    float denom = 1.0f / (va + vb + vc);
    float sv = vb * denom;
    float sw = vc * denom;
    closestX = ax + abx * sv + acx * sw;
    closestY = ay + aby * sv + acy * sw;
    closestZ = az + abz * sv + acz * sw;
    float dx = px - closestX, dy = py - closestY, dz = pz - closestZ;
    return dx * dx + dy * dy + dz * dz;
}

// ─── Build SDF from skinned triangles ──────────────────────────
SMODS_API void SDF_Build(
    int64_t      handle,
    const float* skinnedPos,     // [vertCount*3]
    const float* skinnedNormal,  // [vertCount*3]
    int          vertCount,
    const int*   indices,        // [indexCount] triangle indices (can be null for vertex-only fallback)
    int          indexCount,     // number of indices (multiple of 3)
    float        originX, float originY, float originZ,
    int          resX, int resY, int resZ,
    float        cellSize,
    float        maxDist,
    float        hashCellSize,
    float        queryRadius,
    float*       outSDF)         // [resX*resY*resZ] output
{
    auto* ctx = reinterpret_cast<SDFContext*>(handle);
    if (!ctx) return;

    ctx->resX = resX; ctx->resY = resY; ctx->resZ = resZ;
    ctx->totalVoxels = resX * resY * resZ;
    ctx->originX = originX; ctx->originY = originY; ctx->originZ = originZ;
    ctx->cellSize = cellSize;
    ctx->invCellSize = 1.0f / cellSize;
    ctx->maxDist = maxDist;

    int triCount = (indices != nullptr && indexCount >= 3) ? indexCount / 3 : 0;
    bool useTriangles = triCount > 0;

    // ── Build spatial hash of triangle centroids (or vertices if no triangles) ──
    int hashEntryCount = useTriangles ? triCount : vertCount;
    int ts = SDFPickPrime(std::max(hashEntryCount * 2, 251));
    if (ts > ctx->hashTableCapacity) {
        delete[] ctx->hashCellCount;
        delete[] ctx->hashCellStart;
        ctx->hashCellCount = new int[ts]();
        ctx->hashCellStart = new int[ts]();
        ctx->hashTableCapacity = ts;
    }
    if (hashEntryCount > ctx->hashCapacity) {
        delete[] ctx->hashEntries;
        delete[] ctx->hashPerParticle;
        ctx->hashEntries = new int[hashEntryCount];
        ctx->hashPerParticle = new int[hashEntryCount];
        ctx->hashCapacity = hashEntryCount;
    }
    ctx->hashTableSize = ts;
    std::memset(ctx->hashCellCount, 0, ts * sizeof(int));

    float invHash = 1.0f / hashCellSize;

    if (useTriangles) {
        // Hash triangle centroids
        for (int t = 0; t < triCount; t++) {
            int i0 = indices[t * 3], i1 = indices[t * 3 + 1], i2 = indices[t * 3 + 2];
            float cx = (skinnedPos[i0 * 3] + skinnedPos[i1 * 3] + skinnedPos[i2 * 3]) / 3.0f;
            float cy = (skinnedPos[i0 * 3 + 1] + skinnedPos[i1 * 3 + 1] + skinnedPos[i2 * 3 + 1]) / 3.0f;
            float cz = (skinnedPos[i0 * 3 + 2] + skinnedPos[i1 * 3 + 2] + skinnedPos[i2 * 3 + 2]) / 3.0f;
            int hx = (int)std::floor(cx * invHash);
            int hy = (int)std::floor(cy * invHash);
            int hz = (int)std::floor(cz * invHash);
            int h = (int)SDF_HashCell(hx, hy, hz, ts);
            ctx->hashPerParticle[t] = h;
            ctx->hashCellCount[h]++;
        }
    } else {
        // Hash vertices (legacy fallback)
        for (int i = 0; i < vertCount; i++) {
            int ix = (int)std::floor(skinnedPos[i * 3]     * invHash);
            int iy = (int)std::floor(skinnedPos[i * 3 + 1] * invHash);
            int iz = (int)std::floor(skinnedPos[i * 3 + 2] * invHash);
            int h = (int)SDF_HashCell(ix, iy, iz, ts);
            ctx->hashPerParticle[i] = h;
            ctx->hashCellCount[h]++;
        }
    }

    // Prefix sum
    int running = 0;
    for (int c = 0; c < ts; c++) {
        ctx->hashCellStart[c] = running;
        running += ctx->hashCellCount[c];
        ctx->hashCellCount[c] = 0;
    }

    // Scatter
    for (int i = 0; i < hashEntryCount; i++) {
        int h = ctx->hashPerParticle[i];
        ctx->hashEntries[ctx->hashCellStart[h] + ctx->hashCellCount[h]] = i;
        ctx->hashCellCount[h]++;
    }

    // ── Build SDF ──
    int total = resX * resY * resZ;
    int rxy = resX * resY;

    for (int vi = 0; vi < total; vi++) {
        int vz = vi / rxy;
        int remain = vi - vz * rxy;
        int vy = remain / resX;
        int vx = remain - vy * resX;

        float wpx = originX + vx * cellSize;
        float wpy = originY + vy * cellSize;
        float wpz = originZ + vz * cellSize;

        int minIx = (int)std::floor((wpx - queryRadius) * invHash);
        int maxIx = (int)std::floor((wpx + queryRadius) * invHash);
        int minIy = (int)std::floor((wpy - queryRadius) * invHash);
        int maxIy = (int)std::floor((wpy + queryRadius) * invHash);
        int minIz = (int)std::floor((wpz - queryRadius) * invHash);
        int maxIz = (int)std::floor((wpz + queryRadius) * invHash);

        float bestDistSq = 1e30f;
        float bestClosestX = 0, bestClosestY = 0, bestClosestZ = 0;
        float bestNX = 0, bestNY = 1, bestNZ = 0; // fallback normal

        if (useTriangles) {
            // Triangle-based: find closest triangle
            for (int ix = minIx; ix <= maxIx; ix++) {
                for (int iy = minIy; iy <= maxIy; iy++) {
                    for (int iz = minIz; iz <= maxIz; iz++) {
                        int h = (int)SDF_HashCell(ix, iy, iz, ts);
                        int start = ctx->hashCellStart[h];
                        int count = ctx->hashCellCount[h];
                        for (int k = 0; k < count; k++) {
                            int triIdx = ctx->hashEntries[start + k];
                            int i0 = indices[triIdx * 3];
                            int i1 = indices[triIdx * 3 + 1];
                            int i2 = indices[triIdx * 3 + 2];

                            float closX, closY, closZ;
                            float dSq = PointTriangleDistSq(
                                wpx, wpy, wpz,
                                skinnedPos[i0 * 3], skinnedPos[i0 * 3 + 1], skinnedPos[i0 * 3 + 2],
                                skinnedPos[i1 * 3], skinnedPos[i1 * 3 + 1], skinnedPos[i1 * 3 + 2],
                                skinnedPos[i2 * 3], skinnedPos[i2 * 3 + 1], skinnedPos[i2 * 3 + 2],
                                closX, closY, closZ);

                            if (dSq < bestDistSq) {
                                bestDistSq = dSq;
                                bestClosestX = closX;
                                bestClosestY = closY;
                                bestClosestZ = closZ;
                                // Compute triangle face normal for sign
                                float e1x = skinnedPos[i1 * 3]     - skinnedPos[i0 * 3];
                                float e1y = skinnedPos[i1 * 3 + 1] - skinnedPos[i0 * 3 + 1];
                                float e1z = skinnedPos[i1 * 3 + 2] - skinnedPos[i0 * 3 + 2];
                                float e2x = skinnedPos[i2 * 3]     - skinnedPos[i0 * 3];
                                float e2y = skinnedPos[i2 * 3 + 1] - skinnedPos[i0 * 3 + 1];
                                float e2z = skinnedPos[i2 * 3 + 2] - skinnedPos[i0 * 3 + 2];
                                bestNX = e1y * e2z - e1z * e2y;
                                bestNY = e1z * e2x - e1x * e2z;
                                bestNZ = e1x * e2y - e1y * e2x;
                            }
                        }
                    }
                }
            }
        } else {
            // Vertex-based fallback (legacy K-nearest)
            static const int K_NEAREST = 4;
            float kDistSq[K_NEAREST];
            int   kIdx[K_NEAREST];
            for (int k = 0; k < K_NEAREST; k++) { kDistSq[k] = 1e30f; kIdx[k] = -1; }

            for (int ix = minIx; ix <= maxIx; ix++) {
                for (int iy = minIy; iy <= maxIy; iy++) {
                    for (int iz = minIz; iz <= maxIz; iz++) {
                        int h = (int)SDF_HashCell(ix, iy, iz, ts);
                        int start = ctx->hashCellStart[h];
                        int count = ctx->hashCellCount[h];
                        for (int k = 0; k < count; k++) {
                            int pidx = ctx->hashEntries[start + k];
                            float ddx = wpx - skinnedPos[pidx * 3];
                            float ddy = wpy - skinnedPos[pidx * 3 + 1];
                            float ddz = wpz - skinnedPos[pidx * 3 + 2];
                            float dSq = ddx * ddx + ddy * ddy + ddz * ddz;
                            if (dSq < kDistSq[K_NEAREST - 1]) {
                                kDistSq[K_NEAREST - 1] = dSq;
                                kIdx[K_NEAREST - 1] = pidx;
                                for (int ss = K_NEAREST - 1; ss > 0; ss--) {
                                    if (kDistSq[ss] < kDistSq[ss - 1]) {
                                        std::swap(kDistSq[ss], kDistSq[ss - 1]);
                                        std::swap(kIdx[ss], kIdx[ss - 1]);
                                    }
                                }
                            }
                        }
                    }
                }
            }

            if (kIdx[0] >= 0) {
                bestDistSq = kDistSq[0];
                // Weighted sign voting
                float signAccum = 0.0f;
                float epsilon = kDistSq[0] * 0.01f + 1e-10f;
                for (int nn = 0; nn < K_NEAREST; nn++) {
                    if (kIdx[nn] < 0) break;
                    float tvx = wpx - skinnedPos[kIdx[nn] * 3];
                    float tvy = wpy - skinnedPos[kIdx[nn] * 3 + 1];
                    float tvz = wpz - skinnedPos[kIdx[nn] * 3 + 2];
                    float vote = tvx * skinnedNormal[kIdx[nn] * 3]
                               + tvy * skinnedNormal[kIdx[nn] * 3 + 1]
                               + tvz * skinnedNormal[kIdx[nn] * 3 + 2];
                    float ww = 1.0f / (kDistSq[nn] + epsilon);
                    signAccum += vote * ww;
                }
                outSDF[vi] = (signAccum >= 0.0f) ? std::sqrt(bestDistSq) : -std::sqrt(bestDistSq);
                continue; // skip to next voxel (sign already computed)
            }
        }

        if (bestDistSq >= 1e29f) {
            outSDF[vi] = maxDist;
            continue;
        }

        float dist = std::sqrt(bestDistSq);

        // Sign: dot(voxel - closestPoint, faceNormal)
        float toVoxX = wpx - bestClosestX;
        float toVoxY = wpy - bestClosestY;
        float toVoxZ = wpz - bestClosestZ;
        float signDot = toVoxX * bestNX + toVoxY * bestNY + toVoxZ * bestNZ;
        outSDF[vi] = (signDot >= 0.0f) ? dist : -dist;
    }

    // ── Smoothing pass (3×3×3 Gaussian blur, in-place via temp buffer) ──
    {
        float* tempBuf = new float[total];
        for (int vi2 = 0; vi2 < total; vi2++) {
            int vz2 = vi2 / rxy;
            int rem2 = vi2 - vz2 * rxy;
            int vy2 = rem2 / resX;
            int vx2 = rem2 - vy2 * resX;

            float sum = 0.0f;
            float wTotal = 0.0f;

            for (int dz = -1; dz <= 1; dz++)
            for (int dy = -1; dy <= 1; dy++)
            for (int dx = -1; dx <= 1; dx++) {
                int nx = vx2 + dx, ny = vy2 + dy, nz = vz2 + dz;
                if (nx < 0 || nx >= resX || ny < 0 || ny >= resY || nz < 0 || nz >= resZ)
                    continue;
                int manhattan = std::abs(dx) + std::abs(dy) + std::abs(dz);
                float w;
                if      (manhattan == 0) w = 8.0f;
                else if (manhattan == 1) w = 4.0f;
                else if (manhattan == 2) w = 2.0f;
                else                     w = 1.0f;
                int ni = nz * rxy + ny * resX + nx;
                sum += outSDF[ni] * w;
                wTotal += w;
            }
            tempBuf[vi2] = sum / wTotal;
        }
        std::memcpy(outSDF, tempBuf, total * sizeof(float));
        delete[] tempBuf;
    }
}

// ─── SDF trilinear sample (for batch cloth collision) ──────────
SMODS_API float SDF_Sample(
    const float* sdfData,
    int resX, int resY, int resZ,
    float originX, float originY, float originZ,
    float invCellSize,
    float maxDist,
    float wx, float wy, float wz)
{
    float lx = (wx - originX) * invCellSize;
    float ly = (wy - originY) * invCellSize;
    float lz = (wz - originZ) * invCellSize;

    // Clamp to valid interpolation range instead of returning maxDist at edges.
    // This avoids discontinuous "walls" at the SDF volume boundary.
    lx = std::max(0.0f, std::min(lx, (float)(resX - 1) - 1e-4f));
    ly = std::max(0.0f, std::min(ly, (float)(resY - 1) - 1e-4f));
    lz = std::max(0.0f, std::min(lz, (float)(resZ - 1) - 1e-4f));

    // Still return maxDist if completely outside the padded region
    float lxRaw = (wx - originX) * invCellSize;
    float lyRaw = (wy - originY) * invCellSize;
    float lzRaw = (wz - originZ) * invCellSize;
    if (lxRaw < -1.0f || lyRaw < -1.0f || lzRaw < -1.0f ||
        lxRaw > resX || lyRaw > resY || lzRaw > resZ)
        return maxDist;

    int ix = (int)lx, iy = (int)ly, iz = (int)lz;
    float fx = lx - ix, fy = ly - iy, fz = lz - iz;

    int rxy = resX * resY;
    auto Idx = [&](int x, int y, int z) { return z * rxy + y * resX + x; };

    float c000 = sdfData[Idx(ix,   iy,   iz  )];
    float c100 = sdfData[Idx(ix+1, iy,   iz  )];
    float c010 = sdfData[Idx(ix,   iy+1, iz  )];
    float c110 = sdfData[Idx(ix+1, iy+1, iz  )];
    float c001 = sdfData[Idx(ix,   iy,   iz+1)];
    float c101 = sdfData[Idx(ix+1, iy,   iz+1)];
    float c011 = sdfData[Idx(ix,   iy+1, iz+1)];
    float c111 = sdfData[Idx(ix+1, iy+1, iz+1)];

    float c00 = c000 + (c100 - c000) * fx;
    float c10 = c010 + (c110 - c010) * fx;
    float c01 = c001 + (c101 - c001) * fx;
    float c11 = c011 + (c111 - c011) * fx;

    float c0 = c00 + (c10 - c00) * fy;
    float c1 = c01 + (c11 - c01) * fy;

    return c0 + (c1 - c0) * fz;
}

// ─── Batch SDF collision for cloth vertices ────────────────────
SMODS_API void SDF_CollideVertices(
    float*       pred,          // [n*3] in/out
    const float* invMass,       // [n]
    int          n,
    const float* sdfData,
    int resX, int resY, int resZ,
    float originX, float originY, float originZ,
    float invCellSize,
    float maxDist,
    float thickness)
{
    for (int i = 0; i < n; i++) {
        if (invMass[i] <= 0.0f) continue;
        int i3 = i * 3;

        float dist = SDF_Sample(sdfData, resX, resY, resZ,
                                originX, originY, originZ, invCellSize, maxDist,
                                pred[i3], pred[i3 + 1], pred[i3 + 2]);
        if (dist >= thickness) continue;

        // Gradient via central differences
        float h = 1.0f / invCellSize * 1.5f; // gradientSampleFactor=1.5
        float gdx = SDF_Sample(sdfData, resX, resY, resZ, originX, originY, originZ, invCellSize, maxDist,
                               pred[i3] + h, pred[i3 + 1], pred[i3 + 2])
                   - SDF_Sample(sdfData, resX, resY, resZ, originX, originY, originZ, invCellSize, maxDist,
                               pred[i3] - h, pred[i3 + 1], pred[i3 + 2]);
        float gdy = SDF_Sample(sdfData, resX, resY, resZ, originX, originY, originZ, invCellSize, maxDist,
                               pred[i3], pred[i3 + 1] + h, pred[i3 + 2])
                   - SDF_Sample(sdfData, resX, resY, resZ, originX, originY, originZ, invCellSize, maxDist,
                               pred[i3], pred[i3 + 1] - h, pred[i3 + 2]);
        float gdz = SDF_Sample(sdfData, resX, resY, resZ, originX, originY, originZ, invCellSize, maxDist,
                               pred[i3], pred[i3 + 1], pred[i3 + 2] + h)
                   - SDF_Sample(sdfData, resX, resY, resZ, originX, originY, originZ, invCellSize, maxDist,
                               pred[i3], pred[i3 + 1], pred[i3 + 2] - h);

        float gradLen = std::sqrt(gdx * gdx + gdy * gdy + gdz * gdz);
        float pushAmount = thickness - dist;

        if (gradLen > 1e-6f) {
            float invGL = 1.0f / gradLen;
            pred[i3]     += gdx * invGL * pushAmount;
            pred[i3 + 1] += gdy * invGL * pushAmount;
            pred[i3 + 2] += gdz * invGL * pushAmount;
        } else {
            // Zero gradient: near body centre. Use direction from SDF origin
            // centre to the vertex as fallback push direction.
            float toCx = pred[i3]     - (originX + (resX * 0.5f) / invCellSize);
            float toCy = pred[i3 + 1] - (originY + (resY * 0.5f) / invCellSize);
            float toCz = pred[i3 + 2] - (originZ + (resZ * 0.5f) / invCellSize);
            float toLen = std::sqrt(toCx * toCx + toCy * toCy + toCz * toCz);
            if (toLen > 1e-6f) {
                float inv = pushAmount / toLen;
                pred[i3]     += toCx * inv;
                pred[i3 + 1] += toCy * inv;
                pred[i3 + 2] += toCz * inv;
            } else {
                pred[i3 + 1] += pushAmount; // absolute fallback
            }
        }
    }
}
