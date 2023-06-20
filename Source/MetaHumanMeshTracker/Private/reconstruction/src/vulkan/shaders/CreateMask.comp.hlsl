// Copyright Epic Games, Inc. All Rights Reserved.

#include "CreateMask.comp.h"

[[vk::push_constant]] ConstantBuffer<CreateMaskPushConstants> ubo;

[[vk::binding(0)]] StructuredBuffer<float> inputImage;
[[vk::binding(1)]] RWStructuredBuffer<float> outputImage;

[numthreads(CreateMaskComputeThreadSizeX, CreateMaskComputeThreadSizeY, 1)]
void main(uint3 DTid : SV_DispatchThreadID) {

    const int x = DTid.x;
    const int y = DTid.y;

    const int w = ubo.width;
    const int h = ubo.height;

    if(x >= w || y >= h)
        return;

    // extract channel using StructuredBuffer
    float value = (x > 0 && y > 0 && x < w - 1 && y < h - 1) ? inputImage[y * w + x] : 0.0f;
    outputImage[y * w + x] = (value >= ubo.darkThreshold && value <= ubo.brightThreshold) ? 1.0f : 0.0f;
}
