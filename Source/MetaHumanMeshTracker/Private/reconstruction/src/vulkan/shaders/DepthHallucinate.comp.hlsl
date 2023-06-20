// Copyright Epic Games, Inc. All Rights Reserved.

#include "DepthHallucinate.comp.h"

[[vk::push_constant]] ConstantBuffer<DepthHallucinatePushConstants> ubo;

[[vk::binding(0)]] StructuredBuffer<float> depthImage;
[[vk::binding(1)]] StructuredBuffer<float> heightImage;
[[vk::binding(2)]] RWStructuredBuffer<float> outputImage;

[numthreads(DepthHallucinateComputeThreadSizeX, DepthHallucinateComputeThreadSizeY, 1)]
void main(uint3 DTid : SV_DispatchThreadID) {

    const int x = DTid.x;
    const int y = DTid.y;

    const int w = ubo.width;
    const int h = ubo.height;

    if(x >= w || y >= h)
        return;

    float depth = depthImage[y * w + x];
    if (depth > 0) {
        const float height = heightImage[y * w + x];
        depth += ubo.scale * (height - 0.5);
    }
    outputImage[y * w + x] = depth;
}
