// Copyright Epic Games, Inc. All Rights Reserved.

#include "StereoMatching.comp.h"

[[vk::push_constant]] ConstantBuffer<StereoMatchingPushConstants> ubo;

[[vk::binding(0)]] StructuredBuffer<float> imgLeft;
[[vk::binding(1)]] StructuredBuffer<float> imgRight;
[[vk::binding(2)]] RWStructuredBuffer<float> disparityLeft;

#include "StereoHelpers.hlsl"

[numthreads(StereoMatchingComputeThreadSizeX, StereoMatchingComputeThreadSizeY, 1)]
void main(uint3 DTid : SV_DispatchThreadID) {

    const int x = DTid.x;
    const int y = DTid.y;

    const int w = ubo.width;
    const int h = ubo.height;

    if(x >= w || y >= h)
        return;

    const int KERNEL_RADIUS = 1;

    float disparity = disparityLeft[y * w + x];
    if (disparity != 0) {
        const int disparityIndex = int(floor(disparity));
        const float disparityOffset = disparity - disparityIndex;

        const int startX = max(KERNEL_RADIUS + 1, x + (disparityIndex - ubo.numMatchingOffsets - 1));
        const int endX = min(w - 2 - KERNEL_RADIUS, x + (disparityIndex + ubo.numMatchingOffsets + 1));

        float bestScore = 1;
        float bestTargetX = x + disparity;
        for (int targetX = startX; targetX <= endX; ++targetX) {
            const float score = NormalizedCrossCorrelation(imgLeft, imgRight, w, x, y, targetX + disparityOffset);
            if (score < bestScore) {
                bestScore = score;
                bestTargetX = targetX + disparityOffset;
            }
        }

        if (bestScore < 1) {
            float bestTargetXSubpixel = bestTargetX;
            // quadratic interpolation
            const float score1 = NormalizedCrossCorrelation(imgLeft, imgRight, w, x, y, bestTargetX - 1.0f);
            const float score2 = NormalizedCrossCorrelation(imgLeft, imgRight, w, x, y, bestTargetX);
            const float score3 = NormalizedCrossCorrelation(imgLeft, imgRight, w, x, y, bestTargetX + 1.0f);
            bestTargetXSubpixel = bestTargetX + QuadraticInterpolation(score1, score2, score3);
            disparity = bestTargetXSubpixel - x;
        }
        disparityLeft[y * w + x] = disparity;
    }
}
