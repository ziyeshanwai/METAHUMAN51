// Copyright Epic Games, Inc. All Rights Reserved.

#include "DisparityToDepth.comp.h"

[[vk::push_constant]] ConstantBuffer<DisparityToDepthPushConstants> ubo;

[[vk::binding(0)]] StructuredBuffer<float> inputImage;
[[vk::binding(1)]] RWStructuredBuffer<float> outputImage;

[numthreads(DisparityToDepthComputeThreadSizeX, DisparityToDepthComputeThreadSizeY, 1)]
void main(uint3 DTid : SV_DispatchThreadID) {

    const int x = DTid.x;
    const int y = DTid.y;

    const int w = ubo.width;
    const int h = ubo.height;

    if(x >= w || y >= h)
        return;

    const float disparity = inputImage[y * w + x];
    const float depth = (disparity != 0) ? (ubo.fb / (ubo.offset + ubo.scaleDisparity * disparity)) : 0;
    outputImage[y * w + x] = depth;
}
