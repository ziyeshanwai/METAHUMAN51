// Copyright Epic Games, Inc. All Rights Reserved.

#include "StereoRefine.comp.h"

[[vk::push_constant]] ConstantBuffer<StereoRefinePushConstants> ubo;

[[vk::binding(0)]] StructuredBuffer<float> imgLeft;
[[vk::binding(1)]] StructuredBuffer<float> imgRight;
[[vk::binding(2)]] StructuredBuffer<float> inputDisparity;
[[vk::binding(3)]] RWStructuredBuffer<float> outputDisparity;

#include "StereoHelpers.hlsl"

[numthreads(StereoRefineComputeThreadSizeX, StereoRefineComputeThreadSizeY, 1)]
void main(uint3 DTid : SV_DispatchThreadID) {

    const int x = DTid.x;
    const int y = DTid.y;

    const int w = ubo.width;
    const int h = ubo.height;

    if(x >= w || y >= h)
        return;

    const int KERNEL_RADIUS = 1;

    float disparity = 0;
    if (x > 0 && y > 0 && x < w - 1 && y < h - 1) {
        disparity = inputDisparity[y * w + x];
        if (disparity != 0) {
            const float bestTargetXSubpixel = x + disparity;
            if (bestTargetXSubpixel > 2 && bestTargetXSubpixel < w - 3) { // Note: we could also not use this check, then it will do normalize cross correlation out of bounds in x direction, but given that we don't use the first and last row, this would not result in a memory access error
                const float scorePrev = NormalizedCrossCorrelation(imgLeft, imgRight, w, x, y, bestTargetXSubpixel - float(1));
                const float score = NormalizedCrossCorrelation(imgLeft, imgRight, w, x, y, bestTargetXSubpixel);
                const float scoreNext = NormalizedCrossCorrelation(imgLeft, imgRight, w, x, y, bestTargetXSubpixel + float(1));
                const float offset = QuadraticInterpolation(scorePrev, score, scoreNext);
                const float disparityXM = inputDisparity[y * w + x - 1];
                const float disparityXP = inputDisparity[y * w + x + 1];
                const float disparityYM = inputDisparity[(y - 1) * w + x];
                const float disparityYP = inputDisparity[(y + 1) * w + x];
                const float diffx = abs(disparityXM - disparity) + abs(disparityXP - disparity);
                const float diffy = abs(disparityYM - disparity) + abs(disparityYP - disparity);
                const float wp = PhotometricWeight(scorePrev, score, scoreNext);
                const float wx = exp(- diffx * diffx);
                const float wy = exp(- diffy * diffy);
                const float ds = (wx * (disparityXM + disparityXP) + wy * (disparityYM + disparityYP)) / (2 * (wx + wy));
                const float newDisparity = (wp * (disparity + offset) + ubo.ws * ds) / (wp + ubo.ws);
                if (newDisparity + x >= KERNEL_RADIUS && newDisparity + x < w - KERNEL_RADIUS) {
                    disparity = newDisparity;
                }
            }
        }
    }
    outputDisparity[y * w + x] = disparity;
}
