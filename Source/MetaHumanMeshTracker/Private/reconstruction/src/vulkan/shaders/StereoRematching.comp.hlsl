// Copyright Epic Games, Inc. All Rights Reserved.

#include "StereoRematching.comp.h"

[[vk::push_constant]] ConstantBuffer<StereoRematchingPushConstants> ubo;

[[vk::binding(0)]] StructuredBuffer<float> imgLeft;
[[vk::binding(1)]] StructuredBuffer<float> imgRight;
[[vk::binding(2)]] StructuredBuffer<float> inputDisparity;
[[vk::binding(3)]] RWStructuredBuffer<float> outputDisparity;

#include "StereoHelpers.hlsl"

[numthreads(StereoRematchingComputeThreadSizeX, StereoRematchingComputeThreadSizeY, 1)]
void main(uint3 DTid : SV_DispatchThreadID) {

    const int x = DTid.x;
    const int y = DTid.y;

    const int w = ubo.width;
    const int h = ubo.height;

    if(x >= w || y >= h)
        return;

    const int KERNEL_RADIUS = 1;

    float disparity = inputDisparity[y * w + x];
    if (disparity == 0 && x > 0 && x < w - 1 && y > 0 && y < h - 1) {
        float bestScore = 1;
        for (int searchDy = -1; searchDy <= 1; ++searchDy) {
            for (int searchDx = -1; searchDx <= 1; ++searchDx) {
                if (searchDx == 0 && searchDy == 0) continue;
                const float searchDisparity = inputDisparity[(y + searchDy) * w + (x + searchDx)];
                if (searchDisparity != 0 && ((x + searchDisparity) > 1) && ((x + searchDisparity) < (w - 2))) {
                    const float score = NormalizedCrossCorrelation(imgLeft, imgRight, w, x, y, x + searchDisparity);
                    if (score < bestScore) {
                        bestScore = score;
                        disparity = searchDisparity;
                    }
                }
            }
        }
        if (bestScore < 1 && (x + disparity) >= 2 && (x + disparity) < w - 3) {
            const float scorePrev = NormalizedCrossCorrelation(imgLeft, imgRight, w, x, y, x + disparity - 1.0f);
            const float scoreNext = NormalizedCrossCorrelation(imgLeft, imgRight, w, x, y, x + disparity + 1.0f);
            disparity += QuadraticInterpolation(scorePrev, bestScore, scoreNext);
        }
    }
    outputDisparity[y * w + x] = disparity;
}
