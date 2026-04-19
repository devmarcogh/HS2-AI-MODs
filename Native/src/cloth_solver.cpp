#include "studio_mods_native.h"
#include <cmath>
#include <cstring>
#include <algorithm>
#include <immintrin.h>

// ─── Helpers ────────────────────────────────────────────────────
static inline float fast_rsqrt(float x) {
    __m128 v = _mm_set_ss(x);
    __m128 r = _mm_rsqrt_ss(v);
    // One Newton-Raphson iteration: r = r * (3 - x*r*r) * 0.5
    // Refines from ~11-bit to ~22-bit mantissa precision
    __m128 half  = _mm_set_ss(0.5f);
    __m128 three = _mm_set_ss(3.0f);
    __m128 muls  = _mm_mul_ss(_mm_mul_ss(v, r), r);
    r = _mm_mul_ss(_mm_mul_ss(r, _mm_sub_ss(three, muls)), half);
    return _mm_cvtss_f32(r);
}

// ─── Cloth xPBD Context ────────────────────────────────────────
struct ClothXPBDContext {
    int vertCount;
    int edgeCount;
    int bendCount;
};

SMODS_API int64_t Cloth_Create(int vertCount, int edgeCount, int bendCount) {
    auto* ctx = new ClothXPBDContext();
    ctx->vertCount = vertCount;
    ctx->edgeCount = edgeCount;
    ctx->bendCount = bendCount;
    return reinterpret_cast<int64_t>(ctx);
}

SMODS_API void Cloth_Destroy(int64_t handle) {
    delete reinterpret_cast<ClothXPBDContext*>(handle);
}

// ─── Predict: gravity + position prediction ────────────────────
SMODS_API void Cloth_Predict(
    float*       pos,       // [n*3] in/out
    float*       vel,       // [n*3] in/out
    float*       pred,      // [n*3] out
    const float* invMass,   // [n]
    int          n,
    float        gravity,
    float        dt)
{
    float gdt = gravity * dt;

    for (int i = 0; i < n; i++) {
        int i3 = i * 3;
        if (invMass[i] <= 0.0f) {
            pred[i3]     = pos[i3];
            pred[i3 + 1] = pos[i3 + 1];
            pred[i3 + 2] = pos[i3 + 2];
            continue;
        }
        vel[i3 + 1] += gdt; // gravity on Y
        pred[i3]     = pos[i3]     + vel[i3]     * dt;
        pred[i3 + 1] = pos[i3 + 1] + vel[i3 + 1] * dt;
        pred[i3 + 2] = pos[i3 + 2] + vel[i3 + 2] * dt;
    }
}

// ─── Solve edges (xPBD stretch constraints) ────────────────────
SMODS_API void Cloth_SolveEdges(
    float*       pred,          // [n*3] in/out
    const float* invMass,       // [n]
    const int*   isPinned,      // [n] 0 or 1
    const int*   edges,         // [edgeCount*2] vertex index pairs
    const float* restLen,       // [edgeCount]
    int          edgeCount,
    float        tildedCompliance,
    float        pressScale)
{
    for (int e = 0; e < edgeCount; e++) {
        int i = edges[e * 2];
        int j = edges[e * 2 + 1];

        float wi = isPinned[i] ? 0.0f : invMass[i];
        float wj = isPinned[j] ? 0.0f : invMass[j];
        if (wi == 0.0f && wj == 0.0f) continue;

        float wSum = wi + wj + tildedCompliance;
        if (wSum < 1e-10f) continue;

        int i3 = i * 3, j3 = j * 3;
        float dx = pred[i3]     - pred[j3];
        float dy = pred[i3 + 1] - pred[j3 + 1];
        float dz = pred[i3 + 2] - pred[j3 + 2];

        float distSq = dx * dx + dy * dy + dz * dz;
        if (distSq < 1e-14f) continue;

        float invDist = fast_rsqrt(distSq);
        float dist    = distSq * invDist;
        float C       = dist - restLen[e] * pressScale;
        float lambda  = -C / wSum;

        float cx = dx * invDist * lambda;
        float cy = dy * invDist * lambda;
        float cz = dz * invDist * lambda;

        if (!isPinned[i]) {
            pred[i3]     += wi * cx;
            pred[i3 + 1] += wi * cy;
            pred[i3 + 2] += wi * cz;
        }
        if (!isPinned[j]) {
            pred[j3]     -= wj * cx;
            pred[j3 + 1] -= wj * cy;
            pred[j3 + 2] -= wj * cz;
        }
    }
}

// ─── Solve bends (xPBD bend constraints) ───────────────────────
SMODS_API void Cloth_SolveBends(
    float*       pred,
    const float* invMass,
    const int*   isPinned,
    const int*   bendPairs,    // [bendCount*2]
    const float* restBendLen,  // [bendCount]
    int          bendCount,
    float        tildedCompliance)
{
    for (int b = 0; b < bendCount; b++) {
        int i = bendPairs[b * 2];
        int j = bendPairs[b * 2 + 1];

        float wi = isPinned[i] ? 0.0f : invMass[i];
        float wj = isPinned[j] ? 0.0f : invMass[j];
        if (wi == 0.0f && wj == 0.0f) continue;

        float wSum = wi + wj + tildedCompliance;
        if (wSum < 1e-10f) continue;

        int i3 = i * 3, j3 = j * 3;
        float dx = pred[i3]     - pred[j3];
        float dy = pred[i3 + 1] - pred[j3 + 1];
        float dz = pred[i3 + 2] - pred[j3 + 2];

        float distSq = dx * dx + dy * dy + dz * dz;
        if (distSq < 1e-14f) continue;

        float invDist = fast_rsqrt(distSq);
        float dist    = distSq * invDist;
        float C       = dist - restBendLen[b];
        float lambda  = -C / wSum;

        float cx = dx * invDist * lambda;
        float cy = dy * invDist * lambda;
        float cz = dz * invDist * lambda;

        if (!isPinned[i]) {
            pred[i3]     += wi * cx;
            pred[i3 + 1] += wi * cy;
            pred[i3 + 2] += wi * cz;
        }
        if (!isPinned[j]) {
            pred[j3]     -= wj * cx;
            pred[j3 + 1] -= wj * cy;
            pred[j3 + 2] -= wj * cz;
        }
    }
}

// ─── Collide vertices against capsule/sphere colliders ─────────
// NativeCollider struct is defined in studio_mods_native.h

SMODS_API void Cloth_SolveCollision(
    float*              pred,       // [n*3] in/out
    const float*        invMass,    // [n]
    int                 n,
    const NativeCollider* colliders,
    int                 colCount,
    float               thickness)
{
    for (int i = 0; i < n; i++) {
        if (invMass[i] <= 0.0f) continue;

        int i3 = i * 3;
        float px = pred[i3], py = pred[i3 + 1], pz = pred[i3 + 2];

        for (int c = 0; c < colCount; c++) {
            const NativeCollider& col = colliders[c];

            // Closest point on collider
            float closestX, closestY, closestZ;
            if (!col.isCapsule) {
                closestX = col.cx; closestY = col.cy; closestZ = col.cz;
            } else {
                float wx = px - col.p1x;
                float wy = py - col.p1y;
                float wz = pz - col.p1z;
                float dot = wx * col.vx + wy * col.vy + wz * col.vz;
                float t = (col.vDotV > 1e-8f) ? std::min(1.0f, std::max(0.0f, dot / col.vDotV)) : 0.0f;
                closestX = col.p1x + t * col.vx;
                closestY = col.p1y + t * col.vy;
                closestZ = col.p1z + t * col.vz;
            }

            float dx = px - closestX;
            float dy = py - closestY;
            float dz = pz - closestZ;
            float distSq = dx * dx + dy * dy + dz * dz;
            float dist   = (distSq > 1e-12f) ? distSq * fast_rsqrt(distSq) : 0.0f;
            float minSep  = col.radius + thickness;

            float nx, ny, nz;
            if (dist > 1e-6f) {
                float invD = 1.0f / dist;
                nx = dx * invD; ny = dy * invD; nz = dz * invD;
            } else {
                nx = 0; ny = 1; nz = 0;
            }

            switch (col.magneticMode) {
                case 1: { // Attract
                    float range = col.magneticRange;
                    if (dist < range) {
                        float surfX = closestX + nx * minSep;
                        float surfY = closestY + ny * minSep;
                        float surfZ = closestZ + nz * minSep;
                        float blend = (dist < minSep) ? 1.0f : std::min(1.0f, std::max(0.0f, col.magneticStrength));
                        px += (surfX - px) * blend;
                        py += (surfY - py) * blend;
                        pz += (surfZ - pz) * blend;
                    }
                    break;
                }
                case 0: // Repel
                default: {
                    if (distSq < minSep * minSep) {
                        px = closestX + nx * minSep;
                        py = closestY + ny * minSep;
                        pz = closestZ + nz * minSep;
                    }
                    break;
                }
                // case 2 (Off): skip
            }
        }

        pred[i3] = px; pred[i3 + 1] = py; pred[i3 + 2] = pz;
    }
}

// ─── Commit: derive velocity, apply damping, clamp speed ───────
SMODS_API void Cloth_Commit(
    float*       pos,       // [n*3] in/out (updated to pred)
    float*       vel,       // [n*3] out
    const float* pred,      // [n*3] in
    const float* invMass,   // [n]
    int          n,
    float        invDt,
    float        damp,
    float        maxSpeed)
{
    float maxSpeedSq = maxSpeed * maxSpeed;

    for (int i = 0; i < n; i++) {
        if (invMass[i] <= 0.0f) continue;
        int i3 = i * 3;

        float vx = (pred[i3]     - pos[i3])     * invDt * damp;
        float vy = (pred[i3 + 1] - pos[i3 + 1]) * invDt * damp;
        float vz = (pred[i3 + 2] - pos[i3 + 2]) * invDt * damp;

        float sq = vx * vx + vy * vy + vz * vz;
        if (sq > maxSpeedSq) {
            float s = maxSpeed * fast_rsqrt(sq);
            vx *= s; vy *= s; vz *= s;
        }

        vel[i3] = vx; vel[i3 + 1] = vy; vel[i3 + 2] = vz;
        pos[i3] = pred[i3]; pos[i3 + 1] = pred[i3 + 1]; pos[i3 + 2] = pred[i3 + 2];
    }
}

// ─── Batch LBS skinning for cloth vertices ─────────────────────
SMODS_API void Cloth_SkinVertices(
    const float* bindVerts,    // [n*3] bind-pose positions
    const int*   boneIdx,      // [n*4] bone indices per vertex
    const float* boneW,        // [n*4] bone weights per vertex
    const float* skinMats,     // [boneCount*16] column-major 4x4
    int          n,
    float*       outWorld)     // [n*3] output world positions
{
    for (int i = 0; i < n; i++) {
        float vx = bindVerts[i * 3];
        float vy = bindVerts[i * 3 + 1];
        float vz = bindVerts[i * 3 + 2];

        float rx = 0, ry = 0, rz = 0;

        for (int b = 0; b < 4; b++) {
            float w = boneW[i * 4 + b];
            if (w <= 0.0f) continue;
            int bi = boneIdx[i * 4 + b];
            const float* m = &skinMats[bi * 16];

            rx += w * (m[0] * vx + m[4] * vy + m[8]  * vz + m[12]);
            ry += w * (m[1] * vx + m[5] * vy + m[9]  * vz + m[13]);
            rz += w * (m[2] * vx + m[6] * vy + m[10] * vz + m[14]);
        }

        outWorld[i * 3]     = rx;
        outWorld[i * 3 + 1] = ry;
        outWorld[i * 3 + 2] = rz;
    }
}

// ─── World→Local transform (WriteMesh) ─────────────────────────
SMODS_API void Cloth_WorldToLocal(
    const float* worldPos,  // [n*3]
    const float* w2lMat,    // [16] column-major 4x4
    int          n,
    float*       localPos)  // [n*3] output
{
    for (int i = 0; i < n; i++) {
        float x = worldPos[i * 3];
        float y = worldPos[i * 3 + 1];
        float z = worldPos[i * 3 + 2];

        localPos[i * 3]     = w2lMat[0] * x + w2lMat[4] * y + w2lMat[8]  * z + w2lMat[12];
        localPos[i * 3 + 1] = w2lMat[1] * x + w2lMat[5] * y + w2lMat[9]  * z + w2lMat[13];
        localPos[i * 3 + 2] = w2lMat[2] * x + w2lMat[6] * y + w2lMat[10] * z + w2lMat[14];
    }
}

// ═══════════════════════════════════════════════════════════════════
// VBD (Vertex Block Descent) Cloth Solver
// All substeps × iterations run inside a single C++ call.
// ═══════════════════════════════════════════════════════════════════

struct VBDContext {
    int vertCount;
    int edgeCount;
    int bendCount;

    // Topology (owned, immutable after creation)
    float* invMass;
    int*   edges;       // [edgeCount*2]
    float* restLen;     // [edgeCount]
    int*   bends;       // [bendCount*2]
    float* restBendLen; // [bendCount]

    // Internal working buffers (sized vertCount*3)
    float* pred;
};

SMODS_API int64_t Cloth_CreateVBD(
    int vertCount,
    const float* pos,
    const float* invMass,
    int edgeCount,
    const int* edgeIndices,
    const float* restLen,
    int bendCount,
    const int* bendIndices,
    const float* restBendLen)
{
    auto* ctx = new VBDContext();
    ctx->vertCount = vertCount;
    ctx->edgeCount = edgeCount;
    ctx->bendCount = bendCount;

    ctx->invMass = new float[vertCount];
    std::memcpy(ctx->invMass, invMass, vertCount * sizeof(float));

    ctx->edges = new int[edgeCount * 2];
    ctx->restLen = new float[edgeCount];
    std::memcpy(ctx->edges, edgeIndices, edgeCount * 2 * sizeof(int));
    std::memcpy(ctx->restLen, restLen, edgeCount * sizeof(float));

    ctx->bends = new int[bendCount * 2];
    ctx->restBendLen = new float[bendCount];
    if (bendCount > 0) {
        std::memcpy(ctx->bends, bendIndices, bendCount * 2 * sizeof(int));
        std::memcpy(ctx->restBendLen, restBendLen, bendCount * sizeof(float));
    }

    ctx->pred = new float[vertCount * 3];

    return reinterpret_cast<int64_t>(ctx);
}

SMODS_API void Cloth_DestroyVBD(int64_t handle) {
    auto* ctx = reinterpret_cast<VBDContext*>(handle);
    if (!ctx) return;
    delete[] ctx->invMass;
    delete[] ctx->edges;
    delete[] ctx->restLen;
    delete[] ctx->bends;
    delete[] ctx->restBendLen;
    delete[] ctx->pred;
    delete ctx;
}

// ── Internal: SDF sample (same algorithm as sdf_builder.cpp) ──
static float VBD_SampleSDF(
    const float* sdfData,
    int resX, int resY, int resZ,
    float originX, float originY, float originZ,
    float invCellSize, float maxDist,
    float wx, float wy, float wz)
{
    float lx = (wx - originX) * invCellSize;
    float ly = (wy - originY) * invCellSize;
    float lz = (wz - originZ) * invCellSize;

    float lxRaw = lx, lyRaw = ly, lzRaw = lz;
    if (lxRaw < -1.0f || lyRaw < -1.0f || lzRaw < -1.0f ||
        lxRaw > resX || lyRaw > resY || lzRaw > resZ)
        return maxDist;

    lx = std::max(0.0f, std::min(lx, (float)(resX - 1) - 1e-4f));
    ly = std::max(0.0f, std::min(ly, (float)(resY - 1) - 1e-4f));
    lz = std::max(0.0f, std::min(lz, (float)(resZ - 1) - 1e-4f));

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

SMODS_API void Cloth_StepVBD(
    int64_t handle,
    float* pos,                     // [n*3] in/out
    float* vel,                     // [n*3] in/out
    const NativeCollider* colliders, int colCount,
    const float* sdfData,
    int sdfResX, int sdfResY, int sdfResZ,
    float sdfOriginX, float sdfOriginY, float sdfOriginZ,
    float sdfInvCellSize, float sdfMaxDist, float sdfThickness,
    float dt,
    int   substeps,
    int   iterations,
    float gravity,
    float stretchStiffness,
    float bendStiffness,
    float damping,
    float friction,
    float thickness,
    float maxSpeed,
    float compression,
    const float* invMass
)
{
    auto* ctx = reinterpret_cast<VBDContext*>(handle);
    if (!ctx || ctx->vertCount == 0 || substeps <= 0) return;

    int   n          = ctx->vertCount;
    const float* wInv = (invMass != nullptr) ? invMass : ctx->invMass;
    float* pred      = ctx->pred;
    float  subDt     = dt / (float)substeps;
    float  maxSpeedSq = maxSpeed * maxSpeed;
    float pressScale = 1.0f - compression * 0.35f;
    bool   hasSDF     = sdfData != nullptr && sdfResX > 0 && sdfResY > 0 && sdfResZ > 0;

    for (int sub = 0; sub < substeps; sub++) {
        float gdt = gravity * subDt;

        // ── 1. Predict: p = x + v·dt (gravity into velocity first) ──
        for (int i = 0; i < n; i++) {
            int i3 = i * 3;
            if (wInv[i] <= 0.0f) {
                pred[i3]     = pos[i3];
                pred[i3 + 1] = pos[i3 + 1];
                pred[i3 + 2] = pos[i3 + 2];
                continue;
            }
            vel[i3 + 1] += gdt; // gravity on Y
            pred[i3]     = pos[i3]     + vel[i3]     * subDt;
            pred[i3 + 1] = pos[i3 + 1] + vel[i3 + 1] * subDt;
            pred[i3 + 2] = pos[i3 + 2] + vel[i3 + 2] * subDt;
        }

        // ── 2. VBD iterations: edge + bend + collision all inside loop ──
        float tildedCompl  = 1.0f / std::max(1e-6f, stretchStiffness * subDt * subDt);
        float bendCompl    = 1.0f / std::max(1e-6f, bendStiffness * subDt * subDt);

        for (int iter = 0; iter < iterations; iter++) {

            // ── 2a. Stretch constraints (VBD per-vertex update) ──
            // For each vertex: accumulate gradient from all connected edges
            // then apply a single descent step.
            for (int e = 0; e < ctx->edgeCount; e++) {
                int vi = ctx->edges[e * 2];
                int vj = ctx->edges[e * 2 + 1];

                float wi = wInv[vi];
                float wj = wInv[vj];
                if (wi == 0.0f && wj == 0.0f) continue;

                float wSum = wi + wj + tildedCompl;
                if (wSum < 1e-10f) continue;

                int i3 = vi * 3, j3 = vj * 3;
                float dx = pred[i3]     - pred[j3];
                float dy = pred[i3 + 1] - pred[j3 + 1];
                float dz = pred[i3 + 2] - pred[j3 + 2];

                float distSq = dx * dx + dy * dy + dz * dz;
                if (distSq < 1e-14f) continue;

                float invDist = fast_rsqrt(distSq);
                float dist    = distSq * invDist;
                float C       = dist - ctx->restLen[e] * pressScale;
                float lambda  = -C / wSum;

                float cx = dx * invDist * lambda;
                float cy = dy * invDist * lambda;
                float cz = dz * invDist * lambda;

                if (wi > 0.0f) {
                    pred[i3]     += wi * cx;
                    pred[i3 + 1] += wi * cy;
                    pred[i3 + 2] += wi * cz;
                }
                if (wj > 0.0f) {
                    pred[j3]     -= wj * cx;
                    pred[j3 + 1] -= wj * cy;
                    pred[j3 + 2] -= wj * cz;
                }
            }

            // ── 2b. Bend constraints ──
            for (int b = 0; b < ctx->bendCount; b++) {
                int vi = ctx->bends[b * 2];
                int vj = ctx->bends[b * 2 + 1];

                float wi = wInv[vi];
                float wj = wInv[vj];
                if (wi == 0.0f && wj == 0.0f) continue;

                float wSum = wi + wj + bendCompl;
                if (wSum < 1e-10f) continue;

                int i3 = vi * 3, j3 = vj * 3;
                float dx = pred[i3]     - pred[j3];
                float dy = pred[i3 + 1] - pred[j3 + 1];
                float dz = pred[i3 + 2] - pred[j3 + 2];

                float distSq = dx * dx + dy * dy + dz * dz;
                if (distSq < 1e-14f) continue;

                float invDist = fast_rsqrt(distSq);
                float dist    = distSq * invDist;
                float C       = dist - ctx->restBendLen[b];
                float lambda  = -C / wSum;

                float cx = dx * invDist * lambda;
                float cy = dy * invDist * lambda;
                float cz = dz * invDist * lambda;

                if (wi > 0.0f) {
                    pred[i3]     += wi * cx;
                    pred[i3 + 1] += wi * cy;
                    pred[i3 + 2] += wi * cz;
                }
                if (wj > 0.0f) {
                    pred[j3]     -= wj * cx;
                    pred[j3 + 1] -= wj * cy;
                    pred[j3 + 2] -= wj * cz;
                }
            }

            // ── 2c. Collision (inside iteration loop for stability) ──

            // Capsule/sphere colliders
            for (int i = 0; i < n; i++) {
                if (wInv[i] <= 0.0f) continue;
                int i3 = i * 3;
                float px = pred[i3], py = pred[i3 + 1], pz = pred[i3 + 2];

                for (int c = 0; c < colCount; c++) {
                    const NativeCollider& col = colliders[c];

                    float closestX, closestY, closestZ;
                    if (!col.isCapsule) {
                        closestX = col.cx; closestY = col.cy; closestZ = col.cz;
                    } else {
                        float wx = px - col.p1x, wy = py - col.p1y, wz = pz - col.p1z;
                        float dot = wx * col.vx + wy * col.vy + wz * col.vz;
                        float t = (col.vDotV > 1e-8f)
                            ? std::min(1.0f, std::max(0.0f, dot / col.vDotV)) : 0.0f;
                        closestX = col.p1x + t * col.vx;
                        closestY = col.p1y + t * col.vy;
                        closestZ = col.p1z + t * col.vz;
                    }

                    float ddx = px - closestX;
                    float ddy = py - closestY;
                    float ddz = pz - closestZ;
                    float dSq = ddx * ddx + ddy * ddy + ddz * ddz;
                    float dist = (dSq > 1e-12f) ? dSq * fast_rsqrt(dSq) : 0.0f;
                    float minSep = col.radius + thickness;

                    float nx, ny, nz;
                    if (dist > 1e-6f) {
                        float invD = 1.0f / dist;
                        nx = ddx * invD; ny = ddy * invD; nz = ddz * invD;
                    } else {
                        nx = 0; ny = 1; nz = 0;
                    }

                    switch (col.magneticMode) {
                        case 1: { // Attract
                            float range = col.magneticRange;
                            if (dist < range) {
                                float surfX = closestX + nx * minSep;
                                float surfY = closestY + ny * minSep;
                                float surfZ = closestZ + nz * minSep;
                                float blend = (dist < minSep) ? 1.0f
                                    : std::min(1.0f, std::max(0.0f, col.magneticStrength));
                                px += (surfX - px) * blend;
                                py += (surfY - py) * blend;
                                pz += (surfZ - pz) * blend;
                            }
                            break;
                        }
                        case 0: default: {
                            if (dSq < minSep * minSep) {
                                px = closestX + nx * minSep;
                                py = closestY + ny * minSep;
                                pz = closestZ + nz * minSep;
                            }
                            break;
                        }
                    }
                }
                pred[i3] = px; pred[i3 + 1] = py; pred[i3 + 2] = pz;
            }

            // SDF body collision (integrated into VBD iterations)
            if (hasSDF) {
                for (int i = 0; i < n; i++) {
                    if (wInv[i] <= 0.0f) continue;
                    int i3 = i * 3;

                    float dist = VBD_SampleSDF(sdfData, sdfResX, sdfResY, sdfResZ,
                        sdfOriginX, sdfOriginY, sdfOriginZ,
                        sdfInvCellSize, sdfMaxDist,
                        pred[i3], pred[i3 + 1], pred[i3 + 2]);

                    if (dist >= sdfThickness) continue;

                    // Gradient via central differences
                    float h = (1.0f / sdfInvCellSize) * 1.5f;
                    float gdx = VBD_SampleSDF(sdfData, sdfResX, sdfResY, sdfResZ,
                                    sdfOriginX, sdfOriginY, sdfOriginZ, sdfInvCellSize, sdfMaxDist,
                                    pred[i3] + h, pred[i3 + 1], pred[i3 + 2])
                              - VBD_SampleSDF(sdfData, sdfResX, sdfResY, sdfResZ,
                                    sdfOriginX, sdfOriginY, sdfOriginZ, sdfInvCellSize, sdfMaxDist,
                                    pred[i3] - h, pred[i3 + 1], pred[i3 + 2]);
                    float gdy = VBD_SampleSDF(sdfData, sdfResX, sdfResY, sdfResZ,
                                    sdfOriginX, sdfOriginY, sdfOriginZ, sdfInvCellSize, sdfMaxDist,
                                    pred[i3], pred[i3 + 1] + h, pred[i3 + 2])
                              - VBD_SampleSDF(sdfData, sdfResX, sdfResY, sdfResZ,
                                    sdfOriginX, sdfOriginY, sdfOriginZ, sdfInvCellSize, sdfMaxDist,
                                    pred[i3], pred[i3 + 1] - h, pred[i3 + 2]);
                    float gdz = VBD_SampleSDF(sdfData, sdfResX, sdfResY, sdfResZ,
                                    sdfOriginX, sdfOriginY, sdfOriginZ, sdfInvCellSize, sdfMaxDist,
                                    pred[i3], pred[i3 + 1], pred[i3 + 2] + h)
                              - VBD_SampleSDF(sdfData, sdfResX, sdfResY, sdfResZ,
                                    sdfOriginX, sdfOriginY, sdfOriginZ, sdfInvCellSize, sdfMaxDist,
                                    pred[i3], pred[i3 + 1], pred[i3 + 2] - h);

                    float gradLen = std::sqrt(gdx * gdx + gdy * gdy + gdz * gdz);
                    float pushAmt = sdfThickness - dist;

                    if (gradLen > 1e-6f) {
                        float invGL = 1.0f / gradLen;
                        pred[i3]     += gdx * invGL * pushAmt;
                        pred[i3 + 1] += gdy * invGL * pushAmt;
                        pred[i3 + 2] += gdz * invGL * pushAmt;
                    } else {
                        pred[i3 + 1] += pushAmt; // fallback
                    }

                    // Friction: remove tangential velocity component
                    if (friction > 0.0f && gradLen > 1e-6f) {
                        float invGL = 1.0f / gradLen;
                        float fnx = gdx * invGL, fny = gdy * invGL, fnz = gdz * invGL;
                        float dvx = pred[i3]     - pos[i3];
                        float dvy = pred[i3 + 1] - pos[i3 + 1];
                        float dvz = pred[i3 + 2] - pos[i3 + 2];
                        float normalComp = dvx * fnx + dvy * fny + dvz * fnz;
                        float tx = dvx - normalComp * fnx;
                        float ty = dvy - normalComp * fny;
                        float tz = dvz - normalComp * fnz;
                        pred[i3]     -= tx * friction;
                        pred[i3 + 1] -= ty * friction;
                        pred[i3 + 2] -= tz * friction;
                    }
                }
            }

        } // end iterations

        // ── 3. Commit: derive velocity, damping, clamp, update positions ──
        float invSubDt = 1.0f / subDt;
        float dampFactor = std::max(0.0f, std::min(1.0f, 1.0f - damping * subDt));

        for (int i = 0; i < n; i++) {
            if (wInv[i] <= 0.0f) continue;
            int i3 = i * 3;

            float vx = (pred[i3]     - pos[i3])     * invSubDt * dampFactor;
            float vy = (pred[i3 + 1] - pos[i3 + 1]) * invSubDt * dampFactor;
            float vz = (pred[i3 + 2] - pos[i3 + 2]) * invSubDt * dampFactor;

            float sq = vx * vx + vy * vy + vz * vz;
            if (sq > maxSpeedSq) {
                float s = maxSpeed * fast_rsqrt(sq);
                vx *= s; vy *= s; vz *= s;
            }

            vel[i3] = vx; vel[i3 + 1] = vy; vel[i3 + 2] = vz;
            pos[i3] = pred[i3]; pos[i3 + 1] = pred[i3 + 1]; pos[i3 + 2] = pred[i3 + 2];
        }
    } // end substeps
}
