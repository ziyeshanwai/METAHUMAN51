// Copyright Epic Games, Inc. All Rights Reserved.

#include "CopyBuffer.comp.h"

[[vk::push_constant]] ConstantBuffer<CopyBufferPushConstants> ubo;

[[vk::binding(0)]] StructuredBuffer<float> input;
[[vk::binding(1)]] RWStructuredBuffer<float> output;

[numthreads(CopyBufferThreadSizeX, CopyBufferThreadSizeY, 1)]
void main(const uint3 DTid : SV_DispatchThreadID)
{
    const int x = DTid.x;
    const int y = DTid.y;

    if (x < ubo.inputWidth && y < ubo.inputHeight)
    {
        output[x + y * ubo.outputWidth] = input[x + y * ubo.inputWidth];
    }
}
