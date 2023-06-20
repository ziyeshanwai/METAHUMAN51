// Copyright Epic Games, Inc. All Rights Reserved.

#include "StereoOrderingConstraint.comp.h"

[[vk::push_constant]] ConstantBuffer<StereoOrderingConstraintPushConstants> ubo;

[[vk::binding(0)]] StructuredBuffer<float> inputImage;
[[vk::binding(1)]] RWStructuredBuffer<float> outputImage;

[numthreads(StereoOrderingConstraintComputeThreadSizeX, StereoOrderingConstraintComputeThreadSizeY, 1)]
void main(uint3 DTid : SV_DispatchThreadID) {

    const int x = DTid.x;
    const int y = DTid.y;

    const int w = ubo.width;
    const int h = ubo.height;

    if(x >= w || y >= h)
        return;

    float disparity = inputImage[y * w + x];
    if (disparity != 0 && x < w - 1) {
        const float nextDisparity = inputImage[y * w + x + 1];
        if (nextDisparity != 0 && disparity > nextDisparity + 1) { // otherwise the next pixel would be occluded
            disparity = 0;
        }
    }
    outputImage[y * w + x] = disparity;
}
