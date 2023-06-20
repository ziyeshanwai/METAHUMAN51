// Copyright Epic Games, Inc. All Rights Reserved.

#include "FillBuffer.comp.h"

[[vk::push_constant]] ConstantBuffer<FillBufferPushConstants> ubo;

[[vk::binding(0)]] RWStructuredBuffer<float> buffer;

[numthreads(FillBufferThreadSizeX, FillBufferThreadSizeY, 1)]
void main(const uint3 DTid : SV_DispatchThreadID)
{
    const int x = DTid.x;
    const int y = DTid.y;

    if (x < ubo.width && y < ubo.height)
    {
        buffer[x + y * ubo.width] = ubo.value;
    }
}
