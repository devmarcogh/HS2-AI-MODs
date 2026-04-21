#pragma once
#include <cstdint>

// ─── Export macro ───────────────────────────────────────────────
#ifdef _WIN32
#   define SMODS_API extern "C" __declspec(dllexport)
#else
#   define SMODS_API extern "C" __attribute__((visibility("default")))
#endif

// ─── Interop structs (must match C# layout) ────────────────────

struct Vec3 {
    float x, y, z;
};

struct BoneWeightNative {
    int   idx0, idx1, idx2, idx3;
    float w0,   w1,   w2,   w3;
};

struct Mat4x4 {
    float m[16]; // column-major like Unity
};

// ─── PBD Solver ─────────────────────────────────────────────────

/// Initialise a PBD context. Returns an opaque handle.
SMODS_API int64_t PBD_Create(
    int          sbVertexCount,
    const int*   sbVertexIndices,
    const float* inverseMass,
    int          edgeCount,
    const int*   edgeA,
    const int*   edgeB,
    const float* edgeRestLength
);

/// Run one frame of PBD: integrate forces + solve constraints.
/// physWorldPos is read/written in-place (3 floats per vertex).
/// velocity is read/written in-place (3 floats per vertex).
SMODS_API void PBD_Step(
    int64_t handle,
    float*  physWorldPos,       // [sbCount * 3]  in/out
    float*  velocity,           // [sbCount * 3]  in/out
    const float* animatedPos,   // [sbCount * 3]  in  (forward-skinned rest positions)
    float   dt,
    float   intensity,
    float   stiffness,
    float   damping,
    float   gravitySag,
    float   lateralMul,
    float   secondaryAmp,
    float   refAccelX, float refAccelY, float refAccelZ,
    float   lateralDirX, float lateralDirY, float lateralDirZ,
    float   time,
    int     iterations
);

/// Release a PBD context.
SMODS_API void PBD_Destroy(int64_t handle);

// ─── Spatial Hash ───────────────────────────────────────────────

/// Build a spatial hash grid from positions.
/// Returns an opaque handle.
SMODS_API int64_t SHash_Create(int capacity);

/// Rebuild grid with current positions.
SMODS_API void SHash_Build(
    int64_t      handle,
    const float* positions, // [count * 3]
    int          count,
    float        cellSize
);

/// Query neighbours within radius.
/// Writes indices into outIndices, returns how many found (up to maxResults).
SMODS_API int SHash_Query(
    int64_t      handle,
    float        cx, float cy, float cz,
    float        radius,
    int*         outIndices,
    int          maxResults
);

/// Release a spatial hash context.
SMODS_API void SHash_Destroy(int64_t handle);

// ─── Forward Skinning (batch) ───────────────────────────────────


// ─── Cloth xPBD Solver ──────────────────────────────────────────

/// Create cloth xPBD context. Returns opaque handle.
SMODS_API int64_t Cloth_Create(int vertCount, int edgeCount, int bendCount);

/// Destroy cloth context.
SMODS_API void Cloth_Destroy(int64_t handle);

/// Predict step: integrate gravity into velocity, compute predicted positions.
SMODS_API void Cloth_Predict(
    float* pos, float* vel, float* pred,
    const float* invMass, int n,
    float gravity, float dt);

/// Solve stretch constraints (xPBD). Processes edges sequentially per color group.
/// Call once per color group from C#.
SMODS_API void Cloth_SolveEdges(
    float* pred, const float* invMass, const int* isPinned,
    const int* edges, const float* restLen, int edgeCount,
    float tildedCompliance, float pressScale);

/// Solve bend constraints (xPBD). Call once per color group.
SMODS_API void Cloth_SolveBends(
    float* pred, const float* invMass, const int* isPinned,
    const int* bendPairs, const float* restBendLen, int bendCount,
    float tildedCompliance);

// ─── NativeCollider struct (must match C# StructLayout.Sequential) ──
struct NativeCollider {
    float cx, cy, cz;
    float radius;
    int   isCapsule;
    float p1x, p1y, p1z;
    float vx, vy, vz;
    float vDotV;
    int   magneticMode;
    float magneticStrength;
    float magneticRange;
};

/// Solve capsule/sphere collision for predicted positions.
SMODS_API void Cloth_SolveCollision(
    float* pred, const float* invMass, int n,
    const NativeCollider* colliders, int colCount, float thickness);

/// Commit step: derive velocity from predicted, apply damping and speed clamping.
SMODS_API void Cloth_Commit(
    float* pos, float* vel, const float* pred, const float* invMass,
    int n, float invDt, float damp, float maxSpeed);

/// Batch LBS skinning for cloth vertices.
SMODS_API void Cloth_SkinVertices(
    const float* bindVerts, const int* boneIdx, const float* boneW,
    const float* skinMats, int n, float* outWorld);

/// World→Local transform for WriteMesh.
SMODS_API void Cloth_WorldToLocal(
    const float* worldPos, const float* w2lMat, int n, float* localPos);

// ─── SDF Collider ───────────────────────────────────────────────

/// Create SDF context with pre-allocated buffers.
SMODS_API int64_t SDF_Create(int maxVoxels, int maxVertices);

/// Destroy SDF context.
SMODS_API void SDF_Destroy(int64_t handle);

/// Skin SDF vertices (position + normal) with 4-bone LBS.
SMODS_API void SDF_SkinVertices(
    const float* bindPos, const float* bindNormal,
    const int* boneIdx, const float* boneW,
    const float* boneMats, int count,
    float* outPos, float* outNormal);

/// Build SDF volume from skinned vertices. Includes spatial hash build internally.
SMODS_API void SDF_Build(
    int64_t handle,
    const float* skinnedPos, const float* skinnedNormal, int vertCount,
    float originX, float originY, float originZ,
    int resX, int resY, int resZ,
    float cellSize, float maxDist,
    float hashCellSize, float queryRadius,
    float* outSDF);

/// Sample SDF at a single world-space point (trilinear interpolation).
SMODS_API float SDF_Sample(
    const float* sdfData,
    int resX, int resY, int resZ,
    float originX, float originY, float originZ,
    float invCellSize, float maxDist,
    float wx, float wy, float wz);

/// Batch SDF collision: push cloth vertices out of body using SDF gradient.
SMODS_API void SDF_CollideVertices(
    float* pred, const float* invMass, int n,
    const float* sdfData,
    int resX, int resY, int resZ,
    float originX, float originY, float originZ,
    float invCellSize, float maxDist,
    float thickness,
    float gradientFactor);
