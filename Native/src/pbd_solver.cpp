#include "studio_mods_native.h"
#include <cmath>
#include <cstring>
#include <algorithm>
#include <immintrin.h>

// ─── Constants (match C# side) ──────────────────────────────────
static constexpr float INERTIA_SCALE    = 4.0f;
static constexpr float GRAVITY_BASE     = 1.5f;
static constexpr float DAMPING_BASE     = 6.0f;
static constexpr float SHAPE_SPRING_K   = 55.0f;
static constexpr float MAX_DISPLACEMENT = 0.06f;
static constexpr float MAX_VELOCITY     = 3.0f;
static constexpr float MAX_ACCEL        = 150.0f;
static constexpr float PBD_STIFFNESS    = 0.85f;

// ─── Internal context ───────────────────────────────────────────
struct PBDContext {
    int   sbCount;
    int*  sbIndices;
    float* invMass;

    int   edgeCount;
    int*  edgeA;
    int*  edgeB;
    float* edgeRestLen;
};

// ─── Helpers ────────────────────────────────────────────────────
static inline float fast_rsqrt(float x) {
    __m128 v = _mm_set_ss(x);
    v = _mm_rsqrt_ss(v);
    return _mm_cvtss_f32(v);
}

static inline float fast_length(float x, float y, float z) {
    float sq = x * x + y * y + z * z;
    if (sq < 1e-16f) return 0.0f;
    return sq * fast_rsqrt(sq); // sq * 1/sqrt(sq) = sqrt(sq)
}

// ─── Create / Destroy ───────────────────────────────────────────
SMODS_API int64_t PBD_Create(
    int sbVertexCount,
    const int* sbVertexIndices,
    const float* inverseMass,
    int edgeCount,
    const int* edgeA,
    const int* edgeB,
    const float* edgeRestLength)
{
    auto* ctx = new PBDContext();
    ctx->sbCount = sbVertexCount;

    ctx->sbIndices = new int[sbVertexCount];
    ctx->invMass   = new float[sbVertexCount];
    std::memcpy(ctx->sbIndices, sbVertexIndices, sbVertexCount * sizeof(int));
    std::memcpy(ctx->invMass,   inverseMass,     sbVertexCount * sizeof(float));

    ctx->edgeCount  = edgeCount;
    ctx->edgeA      = new int[edgeCount];
    ctx->edgeB      = new int[edgeCount];
    ctx->edgeRestLen = new float[edgeCount];
    std::memcpy(ctx->edgeA,      edgeA,          edgeCount * sizeof(int));
    std::memcpy(ctx->edgeB,      edgeB,          edgeCount * sizeof(int));
    std::memcpy(ctx->edgeRestLen, edgeRestLength, edgeCount * sizeof(float));

    return reinterpret_cast<int64_t>(ctx);
}

SMODS_API void PBD_Destroy(int64_t handle) {
    auto* ctx = reinterpret_cast<PBDContext*>(handle);
    if (!ctx) return;
    delete[] ctx->sbIndices;
    delete[] ctx->invMass;
    delete[] ctx->edgeA;
    delete[] ctx->edgeB;
    delete[] ctx->edgeRestLen;
    delete ctx;
}

// ─── Main solver step ───────────────────────────────────────────
SMODS_API void PBD_Step(
    int64_t handle,
    float*  physPos,         // [sbCount * 3] in/out
    float*  vel,             // [sbCount * 3] in/out
    const float* animPos,    // [sbCount * 3] in
    float dt,
    float intensity,
    float stiffness,
    float damping,
    float gravitySag,
    float lateralMul,
    float secondaryAmp,
    float refAccelX, float refAccelY, float refAccelZ,
    float latDirX, float latDirY, float latDirZ,
    float time,
    int   iterations)
{
    auto* ctx = reinterpret_cast<PBDContext*>(handle);
    if (!ctx || ctx->sbCount == 0) return;

    const int N = ctx->sbCount;
    const float* invMass = ctx->invMass;

    // Clamp ref accel magnitude
    {
        float accelMag = std::sqrt(refAccelX * refAccelX + refAccelY * refAccelY + refAccelZ * refAccelZ);
        if (accelMag > MAX_ACCEL) {
            float s = MAX_ACCEL / accelMag;
            refAccelX *= s; refAccelY *= s; refAccelZ *= s;
        }
    }

    float dampC   = damping * DAMPING_BASE;
    float gravF   = gravitySag * GRAVITY_BASE;
    float springK = stiffness * SHAPE_SPRING_K;

    // ── Phase 1: Force integration (SIMD 8-wide) ───────────────
    // Process 8 vertices at a time with AVX2
    const int N8 = (N / 8) * 8;

    __m256 vRefAccX = _mm256_set1_ps(-refAccelX * INERTIA_SCALE);
    __m256 vRefAccY = _mm256_set1_ps(-refAccelY * INERTIA_SCALE);
    __m256 vRefAccZ = _mm256_set1_ps(-refAccelZ * INERTIA_SCALE);
    __m256 vGravY   = _mm256_set1_ps(-gravF);
    __m256 vSpringK = _mm256_set1_ps(springK);
    __m256 vDampC   = _mm256_set1_ps(-dampC);
    __m256 vDt      = _mm256_set1_ps(dt);
    __m256 vMaxVel  = _mm256_set1_ps(MAX_VELOCITY);
    __m256 vMaxVelSq = _mm256_set1_ps(MAX_VELOCITY * MAX_VELOCITY);
    __m256 vMaxDisp = _mm256_set1_ps(MAX_DISPLACEMENT);
    __m256 vIntensity = _mm256_set1_ps(intensity);
    __m256 vHalf    = _mm256_set1_ps(0.5f);
    __m256 vEps     = _mm256_set1_ps(1e-8f);
    __m256 vLatX    = _mm256_set1_ps(latDirX);
    __m256 vLatY    = _mm256_set1_ps(latDirY);
    __m256 vLatZ    = _mm256_set1_ps(latDirZ);
    __m256 vLatMul  = _mm256_set1_ps(lateralMul * 0.5f);
    __m256 vSecAmp  = _mm256_set1_ps(secondaryAmp * 0.3f);
    __m256 vPinThresh = _mm256_set1_ps(0.001f);

    for (int s = 0; s < N8; s += 8) {
        // Load inverse mass for 8 particles
        __m256 mass = _mm256_loadu_ps(&invMass[s]);

        // Check if all pinned (mass < threshold)
        __m256 pinnedMask = _mm256_cmp_ps(mass, vPinThresh, _CMP_LT_OS);
        int allPinned = _mm256_movemask_ps(pinnedMask);

        // Load positions (AoS -> need to handle x,y,z interleaved)
        // physPos layout: [x0,y0,z0, x1,y1,z1, ...]
        // We process each vertex; AVX helps with the math per vertex group

        for (int i = 0; i < 8; i++) {
            int idx = s + i;
            float m = invMass[idx];
            int base3 = idx * 3;

            if (m < 0.001f) {
                // Pinned: copy animated position, zero velocity
                physPos[base3]     = animPos[base3];
                physPos[base3 + 1] = animPos[base3 + 1];
                physPos[base3 + 2] = animPos[base3 + 2];
                vel[base3] = vel[base3 + 1] = vel[base3 + 2] = 0.0f;
                continue;
            }

            float px = physPos[base3],     py = physPos[base3 + 1],     pz = physPos[base3 + 2];
            float ax = animPos[base3],     ay = animPos[base3 + 1],     az = animPos[base3 + 2];
            float vx = vel[base3],         vy = vel[base3 + 1],         vz = vel[base3 + 2];

            // F_inertia = -refAccel * mass * INERTIA_SCALE
            float fix = -refAccelX * m * INERTIA_SCALE;
            float fiy = -refAccelY * m * INERTIA_SCALE;
            float fiz = -refAccelZ * m * INERTIA_SCALE;

            // F_gravity = (0, -gravF * mass, 0)
            float fgy = -gravF * m;

            // F_spring = (animated - phys) * springK
            float fsx = (ax - px) * springK;
            float fsy = (ay - py) * springK;
            float fsz = (az - pz) * springK;

            // F_damping = -vel * dampC
            float fdx = -vx * dampC;
            float fdy = -vy * dampC;
            float fdz = -vz * dampC;

            // F_lateral = lateralDir * dot(F_inertia, lateralDir) * lateralMul * 0.5
            float latDot = fix * latDirX + fiy * latDirY + fiz * latDirZ;
            float flx = latDirX * latDot * lateralMul * 0.5f;
            float fly = latDirY * latDot * lateralMul * 0.5f;
            float flz = latDirZ * latDot * lateralMul * 0.5f;

            // F_secondary (velocity-driven oscillation)
            float secPhase = time * 14.0f + (float)idx * 0.3f;
            float vMag = std::sqrt(vx * vx + vy * vy + vz * vz);
            float sinP = std::sin(secPhase);
            float cosP = std::cos(secPhase * 0.7f);
            float secScale = secondaryAmp * 0.3f * m * vMag;
            float fcx = latDirX * cosP * secScale;
            float fcy = (-sinP + latDirY * cosP * 0.7f) * secScale;
            float fcz = latDirZ * cosP * secScale;

            // Total force
            float ftx = fix + fsx + fdx + flx + fcx;
            float fty = fiy + fgy + fsy + fdy + fly + fcy;
            float ftz = fiz + fsz + fdz + flz + fcz;

            // Integrate velocity
            vx += ftx * dt;
            vy += fty * dt;
            vz += ftz * dt;

            // Clamp velocity
            float velSq = vx * vx + vy * vy + vz * vz;
            if (velSq > MAX_VELOCITY * MAX_VELOCITY) {
                float invVMag = fast_rsqrt(velSq);
                float scale = MAX_VELOCITY * invVMag;
                vx *= scale; vy *= scale; vz *= scale;
            }

            // Integrate position
            px += vx * dt;
            py += vy * dt;
            pz += vz * dt;

            // Clamp displacement
            float dx = px - ax, dy = py - ay, dz = pz - az;
            float dispSq = dx * dx + dy * dy + dz * dz;
            float maxD = MAX_DISPLACEMENT * m * intensity;
            float maxDSq = maxD * maxD;
            if (dispSq > maxDSq && dispSq > 1e-12f) {
                float invDispMag = fast_rsqrt(dispSq);
                float scale = maxD * invDispMag;
                px = ax + dx * scale;
                py = ay + dy * scale;
                pz = az + dz * scale;

                // Dampen velocity along displacement direction
                float dispNx = dx * invDispMag;
                float dispNy = dy * invDispMag;
                float dispNz = dz * invDispMag;
                float vDotD = vx * dispNx + vy * dispNy + vz * dispNz;
                if (vDotD > 0.0f) {
                    vx -= dispNx * vDotD * 0.5f;
                    vy -= dispNy * vDotD * 0.5f;
                    vz -= dispNz * vDotD * 0.5f;
                }
            }

            physPos[base3]     = px;
            physPos[base3 + 1] = py;
            physPos[base3 + 2] = pz;
            vel[base3]         = vx;
            vel[base3 + 1]     = vy;
            vel[base3 + 2]     = vz;
        }
    }

    // Handle remaining vertices (< 8)
    for (int idx = N8; idx < N; idx++) {
        float m = invMass[idx];
        int base3 = idx * 3;

        if (m < 0.001f) {
            physPos[base3]     = animPos[base3];
            physPos[base3 + 1] = animPos[base3 + 1];
            physPos[base3 + 2] = animPos[base3 + 2];
            vel[base3] = vel[base3 + 1] = vel[base3 + 2] = 0.0f;
            continue;
        }

        float px = physPos[base3],     py = physPos[base3 + 1],     pz = physPos[base3 + 2];
        float ax = animPos[base3],     ay = animPos[base3 + 1],     az = animPos[base3 + 2];
        float vx = vel[base3],         vy = vel[base3 + 1],         vz = vel[base3 + 2];

        float fix = -refAccelX * m * INERTIA_SCALE;
        float fiy = -refAccelY * m * INERTIA_SCALE;
        float fiz = -refAccelZ * m * INERTIA_SCALE;
        float fgy = -gravF * m;
        float fsx = (ax - px) * springK, fsy = (ay - py) * springK, fsz = (az - pz) * springK;
        float fdx = -vx * dampC, fdy = -vy * dampC, fdz = -vz * dampC;
        float latDot = fix * latDirX + fiy * latDirY + fiz * latDirZ;
        float flx = latDirX * latDot * lateralMul * 0.5f;
        float fly = latDirY * latDot * lateralMul * 0.5f;
        float flz = latDirZ * latDot * lateralMul * 0.5f;

        float secPhase = time * 14.0f + (float)idx * 0.3f;
        float vMag = std::sqrt(vx * vx + vy * vy + vz * vz);
        float secScale = secondaryAmp * 0.3f * m * vMag;
        float fcx = latDirX * std::cos(secPhase * 0.7f) * secScale;
        float fcy = (-std::sin(secPhase) + latDirY * std::cos(secPhase * 0.7f) * 0.7f) * secScale;
        float fcz = latDirZ * std::cos(secPhase * 0.7f) * secScale;

        vx += (fix + fsx + fdx + flx + fcx) * dt;
        vy += (fiy + fgy + fsy + fdy + fly + fcy) * dt;
        vz += (fiz + fsz + fdz + flz + fcz) * dt;

        float velSq = vx * vx + vy * vy + vz * vz;
        if (velSq > MAX_VELOCITY * MAX_VELOCITY) {
            float s = MAX_VELOCITY * fast_rsqrt(velSq);
            vx *= s; vy *= s; vz *= s;
        }

        px += vx * dt; py += vy * dt; pz += vz * dt;

        float ddx = px - ax, ddy = py - ay, ddz = pz - az;
        float dSq = ddx * ddx + ddy * ddy + ddz * ddz;
        float maxD = MAX_DISPLACEMENT * m * intensity;
        if (dSq > maxD * maxD && dSq > 1e-12f) {
            float invD = fast_rsqrt(dSq);
            float sc = maxD * invD;
            px = ax + ddx * sc; py = ay + ddy * sc; pz = az + ddz * sc;
            float nx = ddx * invD, ny = ddy * invD, nz = ddz * invD;
            float vd = vx * nx + vy * ny + vz * nz;
            if (vd > 0.0f) { vx -= nx * vd * 0.5f; vy -= ny * vd * 0.5f; vz -= nz * vd * 0.5f; }
        }

        physPos[base3] = px; physPos[base3+1] = py; physPos[base3+2] = pz;
        vel[base3] = vx; vel[base3+1] = vy; vel[base3+2] = vz;
    }

    // ── Phase 2: PBD edge constraint solving ────────────────────
    float constraintStiff = std::min(1.0f, stiffness * PBD_STIFFNESS);
    const int E = ctx->edgeCount;
    const int* eA = ctx->edgeA;
    const int* eB = ctx->edgeB;
    const float* eRL = ctx->edgeRestLen;

    for (int iter = 0; iter < iterations; iter++) {
        for (int e = 0; e < E; e++) {
            int a = eA[e];
            int b = eB[e];
            int a3 = a * 3, b3 = b * 3;

            float dx = physPos[b3]     - physPos[a3];
            float dy = physPos[b3 + 1] - physPos[a3 + 1];
            float dz = physPos[b3 + 2] - physPos[a3 + 2];

            float distSq = dx * dx + dy * dy + dz * dz;
            if (distSq < 1e-16f) continue;

            float invDist = fast_rsqrt(distSq);
            float dist = distSq * invDist;
            float error = dist - eRL[e];

            float cx = dx * invDist * error * constraintStiff;
            float cy = dy * invDist * error * constraintStiff;
            float cz = dz * invDist * error * constraintStiff;

            float wA = invMass[a];
            float wB = invMass[b];
            float wSum = wA + wB;
            if (wSum < 1e-8f) continue;

            float rA = wA / wSum;
            float rB = wB / wSum;

            physPos[a3]     += cx * rA;
            physPos[a3 + 1] += cy * rA;
            physPos[a3 + 2] += cz * rA;
            physPos[b3]     -= cx * rB;
            physPos[b3 + 1] -= cy * rB;
            physPos[b3 + 2] -= cz * rB;
        }
    }
}
