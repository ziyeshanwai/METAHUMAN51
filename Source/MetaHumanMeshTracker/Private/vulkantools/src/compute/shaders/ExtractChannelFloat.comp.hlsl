// Copyright Epic Games, Inc. All Rights Reserved.

#include "ExtractChannelFloat.comp.h"

[[vk::push_constant]] ConstantBuffer<ExtractChannelFloatPushConstants> ubo;

// StructuredBuffer implementation
[[vk::binding(0)]] StructuredBuffer<float> inputImage;
[[vk::binding(1)]] RWStructuredBuffer<float> outputImage;

// RWTexture2D implementation
// [[vk::binding(0, 0)]] RWTexture2D<float> inputImage;
// [[vk::binding(0, 1)]] RWTexture2D<float> outputImage;

#include "HelperFunctions.h"

[numthreads(ExtractChannelFloatComputeThreadSizeX, ExtractChannelFloatComputeThreadSizeY, 1)]
void main(uint3 DTid : SV_DispatchThreadID) {

    const int x = DTid.x;
    const int y = DTid.y;

    const int w = ubo.width;
    const int h = ubo.height;

    if(x >= w || y >= h)
        return;

    // extract channel using StructuredBuffer
    const int inputIndex = (y * ubo.width + x) * ubo.numChannels + ubo.channel;
    const int outputIndex = y * ubo.width + x;
    if (ubo.isSRGB > 0) {
        outputImage[outputIndex] = srgb2linear(inputImage[inputIndex]  * ubo.scale);
    } else {
        outputImage[outputIndex] = inputImage[inputIndex] * ubo.scale;;
    }

    // extractshannel using RWTexture2D
    // float value = inputImage[int2(x * ubo.numChannels + ubo.channel, y)];
    // outputImage[int2(x, y)] = value;
}
