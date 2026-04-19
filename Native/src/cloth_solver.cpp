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
