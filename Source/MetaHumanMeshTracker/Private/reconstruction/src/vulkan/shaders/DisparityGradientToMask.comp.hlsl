// Copyright Epic Games, Inc. All Rights Reserved.

#include "DisparityGradientToMask.comp.h"

[[vk::push_constant]] ConstantBuffer<DisparityGradientToMaskPushConstants> ubo;

[[vk::binding(0)]] StructuredBuffer<float> inputImage;
[[vk::binding(1)]] RWStructuredBuffer<float> outputImage;

[numthreads(DisparityGradientToMaskComputeThreadSizeX, DisparityGradientToMaskComputeThreadSizeY, 1)]
void main(uint3 DTid : SV_DispatchThreadID) {

    const int x = DTid.x;
    const int y = DTid.y;

    const int w = ubo.width;
    const int h = ubo.height;

    if(x >= w || y >= h)
        return;

    float disparity = inputImage[y * w + x];
    if (disparity != 0) {
        float mag = 0;
        if (x < w - 2) {
            float dx = inputImage[y * w + x + 1] - disparity;
            mag += dx * dx;
        }
        if (y < h -2) {
            float dy = inputImage[(y + 1) * w + x] - disparity;
            mag += dy * dy;
        }
        outputImage[y * w + x] = sqrt(mag) > ubo.threshold ? 0 : 1;
    } else {
        outputImage[y * w + x] = 0;
    }
}
