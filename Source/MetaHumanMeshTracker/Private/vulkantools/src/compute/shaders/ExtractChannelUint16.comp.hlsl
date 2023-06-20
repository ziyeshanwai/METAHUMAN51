// Copyright Epic Games, Inc. All Rights Reserved.

#include "ExtractChannelUint16.comp.h"

[[vk::push_constant]] ConstantBuffer<ExtractChannelUint16PushConstants> ubo;

// StructuredBuffer implementation
[[vk::binding(0)]] StructuredBuffer<uint> inputImage;
[[vk::binding(1)]] RWStructuredBuffer<float> outputImage;

#include "HelperFunctions.h"

[numthreads(ExtractChannelUint16ComputeThreadSizeX, ExtractChannelUint16ComputeThreadSizeY, 1)]
void main(uint3 DTid : SV_DispatchThreadID) {

    const int x = DTid.x;
    const int y = DTid.y;

    const int w = ubo.width;
    const int h = ubo.height;

    if(x >= w || y >= h)
        return;

    const int inputIndex = (y * ubo.width + x) * ubo.numChannels + ubo.channel;
    const int intIndex = inputIndex / 2;
    const int uint16Index = (inputIndex % 2);
    const uint shiftedValue = (inputImage[intIndex] >> (uint16Index * 16)) & 0xFFFF;
    const float floatValue = float(shiftedValue) * ubo.scale;
    //float(1.0/65535.0)

    if (ubo.isSRGB > 0) {
        outputImage[y * w + x] = srgb2linear(floatValue);
    } else {
        outputImage[y * w + x] = floatValue;
    }
}
