// Copyright Epic Games, Inc. All Rights Reserved.

#include "Sobel.comp.h"

[[vk::push_constant]] ConstantBuffer<SobelPushConstants> ubo;

[[vk::binding(0)]] StructuredBuffer<float> input;
[[vk::binding(1)]] RWStructuredBuffer<float> output;

[numthreads(SobelComputeThreadSizeX, SobelComputeThreadSizeY, 1)]
void main(const uint3 DTid : SV_DispatchThreadID)
{
    const int x = DTid.x;
    const int y = DTid.y;

    if (x < ubo.width && y < ubo.height)
    {
        float valueA;
        float valueB;
        switch (ubo.direction)
        {
            case SOBEL_DIRECTION_HORIZONTAL:
                valueA = input[max(x - 1, 0) + y * ubo.width];
                valueB = input[min(x + 1, ubo.width - 1) + y * ubo.width];
                break;
            case SOBEL_DIRECTION_VERTICAL:
                valueA = input[x + max(y - 1, 0) * ubo.width];
                valueB = input[x + min(y + 1, ubo.height - 1) * ubo.width];
                break;
            default:
                abort();
        }
        output[x + y * ubo.width] = valueB - valueA;
    }
}
