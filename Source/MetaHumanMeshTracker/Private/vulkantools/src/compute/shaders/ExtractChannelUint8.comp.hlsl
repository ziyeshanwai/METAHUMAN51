// Copyright Epic Games, Inc. All Rights Reserved.

#include "ExtractChannelUint8.comp.h"

[[vk::push_constant]] ConstantBuffer<ExtractChannelUint8PushConstants> ubo;

// StructuredBuffer implementation
[[vk::binding(0)]] StructuredBuffer<uint> inputImage;
[[vk::binding(1)]] RWStructuredBuffer<float> outputImage;

#include "HelperFunctions.h"

[numthreads(ExtractChannelUint8ComputeThreadSizeX, ExtractChannelUint8ComputeThreadSizeY, 1)]
void main(uint3 DTid : SV_DispatchThreadID) {

    const int x = DTid.x;
    const int y = DTid.y;

    const int w = ubo.width;
    const int h = ubo.height;

    if(x >= w || y >= h)
        return;

    const int inputIndex = (y * ubo.width + x) * ubo.numChannels + ubo.channel;
    const int intIndex = inputIndex / 4;
    const int uint8Index = (inputIndex % 4);
    const uint shiftedValue = (inputImage[intIndex] >> (uint8Index * 8)) & 0xFF;
    const float floatValue = float(shiftedValue) * ubo.scale;

    if (ubo.isSRGB > 0) {
        outputImage[y * w + x] = srgb2linear(floatValue);
    } else {
        outputImage[y * w + x] = floatValue;
    }
}
