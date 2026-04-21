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

// ─── Build SDF from skinned vertices ───────────────────────────
SMODS_API void SDF_Build(
    int64_t      handle,
    const float* skinnedPos,     // [vertCount*3]
    const float* skinnedNormal,  // [vertCount*3]
    int          vertCount,
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

    // ── Build spatial hash of skinned positions ──
    int ts = SDFPickPrime(std::max(vertCount * 2, 251));
    if (ts > ctx->hashTableCapacity) {
        delete[] ctx->hashCellCount;
        delete[] ctx->hashCellStart;
        ctx->hashCellCount = new int[ts]();
        ctx->hashCellStart = new int[ts]();
        ctx->hashTableCapacity = ts;
    }
    if (vertCount > ctx->hashCapacity) {
        delete[] ctx->hashEntries;
        delete[] ctx->hashPerParticle;
        ctx->hashEntries = new int[vertCount];
        ctx->hashPerParticle = new int[vertCount];
        ctx->hashCapacity = vertCount;
    }
    ctx->hashTableSize = ts;

    // Clear
    std::memset(ctx->hashCellCount, 0, ts * sizeof(int));

    float invHash = 1.0f / hashCellSize;

    // Pass 1: count
    for (int i = 0; i < vertCount; i++) {
        int ix = (int)std::floor(skinnedPos[i * 3]     * invHash);
        int iy = (int)std::floor(skinnedPos[i * 3 + 1] * invHash);
        int iz = (int)std::floor(skinnedPos[i * 3 + 2] * invHash);
        int h = (int)SDF_HashCell(ix, iy, iz, ts);
        ctx->hashPerParticle[i] = h;
        ctx->hashCellCount[h]++;
    }

    // Prefix sum
    int running = 0;
    for (int c = 0; c < ts; c++) {
        ctx->hashCellStart[c] = running;
        running += ctx->hashCellCount[c];
        ctx->hashCellCount[c] = 0;
    }

    // Pass 2: scatter
    for (int i = 0; i < vertCount; i++) {
        int h = ctx->hashPerParticle[i];
        ctx->hashEntries[ctx->hashCellStart[h] + ctx->hashCellCount[h]] = i;
        ctx->hashCellCount[h]++;
    }

    // ── Build SDF: for each voxel find closest vertex ──
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

        // Query spatial hash neighbourhood
        int minIx = (int)std::floor((wpx - queryRadius) * invHash);
        int maxIx = (int)std::floor((wpx + queryRadius) * invHash);
        int minIy = (int)std::floor((wpy - queryRadius) * invHash);
        int maxIy = (int)std::floor((wpy + queryRadius) * invHash);
        int minIz = (int)std::floor((wpz - queryRadius) * invHash);
        int maxIz = (int)std::floor((wpz + queryRadius) * invHash);

        // ── K-nearest for sign voting (matches GPU kernel) ──
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
                        float dx = wpx - skinnedPos[pidx * 3];
                        float dy = wpy - skinnedPos[pidx * 3 + 1];
                        float dz = wpz - skinnedPos[pidx * 3 + 2];
                        float dSq = dx * dx + dy * dy + dz * dz;
                        // Insert into K-nearest if closer than farthest
                        if (dSq < kDistSq[K_NEAREST - 1]) {
                            kDistSq[K_NEAREST - 1] = dSq;
                            kIdx[K_NEAREST - 1] = pidx;
                            // Bubble sort towards front
                            for (int s = K_NEAREST - 1; s > 0; s--) {
                                if (kDistSq[s] < kDistSq[s - 1]) {
                                    std::swap(kDistSq[s], kDistSq[s - 1]);
                                    std::swap(kIdx[s], kIdx[s - 1]);
                                }
                            }
                        }
                    }
                }
            }
        }

        if (kIdx[0] < 0) {
            outSDF[vi] = maxDist;
            continue;
        }

        float dist = std::sqrt(kDistSq[0]);

        // Distance-weighted sign voting from K nearest vertices
        float signAccum = 0.0f;
        float epsilon = kDistSq[0] * 0.01f + 1e-10f;
        for (int n = 0; n < K_NEAREST; n++) {
            if (kIdx[n] < 0) break;
            float toVoxelX = wpx - skinnedPos[kIdx[n] * 3];
            float toVoxelY = wpy - skinnedPos[kIdx[n] * 3 + 1];
            float toVoxelZ = wpz - skinnedPos[kIdx[n] * 3 + 2];
            float vote = toVoxelX * skinnedNormal[kIdx[n] * 3]
                       + toVoxelY * skinnedNormal[kIdx[n] * 3 + 1]
                       + toVoxelZ * skinnedNormal[kIdx[n] * 3 + 2];
            float weight = 1.0f / (kDistSq[n] + epsilon);
            signAccum += vote * weight;
        }

        outSDF[vi] = (signAccum >= 0.0f) ? dist : -dist;
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

    // Early return limpio — sin clamp parcial que genera ghost collisions
    if (lx < 0.f || ly < 0.f || lz < 0.f ||
        lx >= (float)(resX - 1) || ly >= (float)(resY - 1) || lz >= (float)(resZ - 1))
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
    float c0  = c00  + (c10  - c00)  * fy;
    float c1  = c01  + (c11  - c01)  * fy;
    return c0 + (c1 - c0) * fz;
}


// ─── Batch SDF collision for cloth vertices ────────────────────
SMODS_API void SDF_CollideVertices(
    float*       pred,
    const float* invMass,
    int          n,
    const float* sdfData,
    int resX, int resY, int resZ,
    float originX, float originY, float originZ,
    float invCellSize,
    float maxDist,
    float thickness,
    float gradientFactor)   // <-- parámetro nuevo (antes hardcodeado a 1.5)
{
    int rxy = resX * resY;
    auto Idx = [&](int x, int y, int z) { return z * rxy + y * resX + x; };

    for (int i = 0; i < n; i++) {
        if (invMass[i] <= 0.0f) continue;
        int i3 = i * 3;
        float wx = pred[i3], wy = pred[i3 + 1], wz = pred[i3 + 2];

        float lx = (wx - originX) * invCellSize;
        float ly = (wy - originY) * invCellSize;
        float lz = (wz - originZ) * invCellSize;

        if (lx < 0.f || ly < 0.f || lz < 0.f ||
            lx >= (float)(resX - 1) || ly >= (float)(resY - 1) || lz >= (float)(resZ - 1))
            continue;

        int ix = (int)lx, iy = (int)ly, iz = (int)lz;
        float fx = lx - ix, fy = ly - iy, fz = lz - iz;

        // Leer los 8 voxels del cubo una sola vez
        float c000 = sdfData[Idx(ix,   iy,   iz  )];
        float c100 = sdfData[Idx(ix+1, iy,   iz  )];
        float c010 = sdfData[Idx(ix,   iy+1, iz  )];
        float c110 = sdfData[Idx(ix+1, iy+1, iz  )];
        float c001 = sdfData[Idx(ix,   iy,   iz+1)];
        float c101 = sdfData[Idx(ix+1, iy,   iz+1)];
        float c011 = sdfData[Idx(ix,   iy+1, iz+1)];
        float c111 = sdfData[Idx(ix+1, iy+1, iz+1)];

        // Distancia trilinear
        float c00  = c000 + (c100 - c000) * fx;
        float c10  = c010 + (c110 - c010) * fx;
        float c01  = c001 + (c101 - c001) * fx;
        float c11  = c011 + (c111 - c011) * fx;
        float c0   = c00  + (c10  - c00)  * fy;
        float c1   = c01  + (c11  - c01)  * fy;
        float dist = c0   + (c1   - c0)   * fz;

        if (dist >= thickness) continue;

        // Gradiente analítico del trilinear — sin samples adicionales, sin cruzar Voronoi
        float gdx = invCellSize * (
            (1.f-fy)*(1.f-fz)*(c100-c000) + fy*(1.f-fz)*(c110-c010) +
            (1.f-fy)*     fz *(c101-c001) + fy*      fz *(c111-c011));
        float gdy = invCellSize * (
            (1.f-fx)*(1.f-fz)*(c010-c000) + fx*(1.f-fz)*(c110-c100) +
            (1.f-fx)*     fz *(c011-c001) + fx*      fz *(c111-c101));
        float gdz = invCellSize * (
            (1.f-fx)*(1.f-fy)*(c001-c000) + fx*(1.f-fy)*(c101-c100) +
            (1.f-fx)*     fy *(c011-c010) + fx*      fy *(c111-c110));

        float gradLen = std::sqrt(gdx*gdx + gdy*gdy + gdz*gdz);
        float pushAmount = thickness - dist;

        if (gradLen > 1e-6f) {
            float invGL = 1.0f / gradLen;
            pred[i3]     += gdx * invGL * pushAmount;
            pred[i3 + 1] += gdy * invGL * pushAmount;
            pred[i3 + 2] += gdz * invGL * pushAmount;
        } else {
            pred[i3 + 1] += pushAmount;
        }
    }
}
