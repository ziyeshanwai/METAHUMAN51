// Copyright Epic Games, Inc. All Rights Reserved.

#include "StereoInitialMatching.comp.h"

[[vk::push_constant]] ConstantBuffer<StereoInitialMatchingPushConstants> ubo;

[[vk::binding(0)]] StructuredBuffer<float> imgLeft;
[[vk::binding(1)]] StructuredBuffer<float> imgRight;
[[vk::binding(2)]] RWStructuredBuffer<float> disparityLeft;

#include "StereoHelpers.hlsl"

[numthreads(StereoInitialMatchingComputeThreadSizeX, StereoInitialMatchingComputeThreadSizeY, 1)]
void main(uint3 DTid : SV_DispatchThreadID) {

    const int x = DTid.x;
    const int y = DTid.y;

    const int w = ubo.width;
    const int h = ubo.height;

    if(x >= w || y >= h)
        return;

    const int KERNEL_RADIUS = 2;

    float disparity = 0;
    float bestScore = 1;
    if (x > KERNEL_RADIUS && x < w - KERNEL_RADIUS && y > KERNEL_RADIUS && y < h - KERNEL_RADIUS) {
        const int startX = max(KERNEL_RADIUS, x + ubo.minimumDisparity);
        const int endX = min(w - 1 - KERNEL_RADIUS, x + ubo.maximumDisparity);
        int bestTargetX = x;
        for (int targetX = startX; targetX <= endX; ++targetX) {
            const float score = NormalizedCrossCorrelationInteger(KERNEL_RADIUS, imgLeft, imgRight, w, x, y, targetX);
            if (score < bestScore) {
                bestScore = score;
                bestTargetX = targetX;
            }
        }

        if (bestScore < 1) {
            // refine the match
            float bestTargetXSubpixel = bestTargetX;
            if (bestTargetX > startX && bestTargetX < endX) {
                // quadratic interpolation
                const float v1 = NormalizedCrossCorrelationInteger(KERNEL_RADIUS, imgLeft, imgRight, w, x, y, bestTargetX - 1);
                const float v2 = NormalizedCrossCorrelationInteger(KERNEL_RADIUS, imgLeft, imgRight, w, x, y, bestTargetX);
                const float v3 = NormalizedCrossCorrelationInteger(KERNEL_RADIUS, imgLeft, imgRight, w, x, y, bestTargetX + 1);
                bestTargetXSubpixel = bestTargetX + QuadraticInterpolation(v1, v2, v3);
            }
            disparity = bestTargetXSubpixel - x;
        }
    }

    disparityLeft[y * w + x] = disparity;
}
