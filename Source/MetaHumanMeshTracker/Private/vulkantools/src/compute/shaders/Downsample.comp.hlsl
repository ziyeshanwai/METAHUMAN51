// Copyright Epic Games, Inc. All Rights Reserved.

#include "Downsample.comp.h"

[[vk::push_constant]] ConstantBuffer<DownsamplePushConstants> ubo;

// StructuredBuffer implementation
[[vk::binding(0)]] StructuredBuffer<float> inputImage;
[[vk::binding(1)]] RWStructuredBuffer<float> outputImage;

[numthreads(DownsampleComputeThreadSizeX, DownsampleComputeThreadSizeY, 1)]
void main(uint3 DTid : SV_DispatchThreadID) {

    const int x = DTid.x;
    const int y = DTid.y;

    const int w = ubo.width;
    const int h = ubo.height;

    if(x >= w || y >= h)
        return;

    // extract channel using StructuredBuffer
    const int inputW = 2 * w;
    const int inputIndex = 2 * y * inputW + 2 * x;
    const int outputIndex = y * w + x;
    outputImage[outputIndex] = 0.25f * (inputImage[inputIndex] + inputImage[inputIndex + 1] + inputImage[inputIndex + inputW] + inputImage[inputIndex + inputW + 1]);
}
