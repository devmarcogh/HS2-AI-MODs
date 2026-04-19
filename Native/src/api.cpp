#include "studio_mods_native.h"
#include <immintrin.h>

// ─── Batch forward skinning ────────────────────────────────────
// Skins only the soft-body vertices, not the whole mesh.
// Each vertex blends up to 4 bone matrices.
// Compiled with /arch:AVX2 for auto-vectorisation; unrolled for ILP.

SMODS_API void Skin_ForwardBatch(
    const Vec3*             bindVerts,
    const BoneWeightNative* boneWeights,
    const Mat4x4*           skinMats,
    const int*              sbIndices,
    int                     sbCount,
    float*                  outWorldPos)
{
    for (int s = 0; s < sbCount; s++) {
        int vi = sbIndices[s];
        const Vec3& v = bindVerts[vi];
        const BoneWeightNative& bw = boneWeights[vi];

        float rx = 0, ry = 0, rz = 0;

        // Unrolled 4-bone blend with direct matrix multiply
        auto skinOne = [&](int boneIdx, float weight) {
            if (weight <= 0.0f) return;
            const float* m = skinMats[boneIdx].m;
            // Column-major multiply: result = M * [vx, vy, vz, 1]
            rx += weight * (m[0] * v.x + m[4] * v.y + m[8]  * v.z + m[12]);
            ry += weight * (m[1] * v.x + m[5] * v.y + m[9]  * v.z + m[13]);
            rz += weight * (m[2] * v.x + m[6] * v.y + m[10] * v.z + m[14]);
        };

        skinOne(bw.idx0, bw.w0);
        skinOne(bw.idx1, bw.w1);
        skinOne(bw.idx2, bw.w2);
        skinOne(bw.idx3, bw.w3);

        outWorldPos[s * 3]     = rx;
        outWorldPos[s * 3 + 1] = ry;
        outWorldPos[s * 3 + 2] = rz;
    }
}
