// Copyright Epic Games, Inc. All Rights Reserved.

#include "StereoUpsample.comp.h"

[[vk::push_constant]] ConstantBuffer<StereoUpsamplePushConstants> ubo;

[[vk::binding(0)]] StructuredBuffer<float> inputImage;
[[vk::binding(1)]] RWStructuredBuffer<float> outputImage;

[numthreads(StereoUpsampleComputeThreadSizeX, StereoUpsampleComputeThreadSizeY, 1)]
void main(uint3 DTid : SV_DispatchThreadID) {

    const int x = DTid.x;
    const int y = DTid.y;

    const int w = ubo.width;
    const int h = ubo.height;

    if(x >= w || y >= h)
        return;

    outputImage[y * w + x] = 2.0f * inputImage[y/2 * w/2 + x/2];
}
