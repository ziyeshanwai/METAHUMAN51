// Copyright Epic Games, Inc. All Rights Reserved.

#include "SplitCheckerboard.comp.h"

[[vk::push_constant]] ConstantBuffer<SplitCheckerboardPushConstants> ubo;

[[vk::binding(0)]] StructuredBuffer<float> input;
[[vk::binding(1)]] RWStructuredBuffer<float> redOutput;
[[vk::binding(2)]] RWStructuredBuffer<float> blackOutput;

[numthreads(SplitCheckerboardThreadSizeX, 1, 1)]
void main(const uint3 DTid : SV_DispatchThreadID)
{
    const int x = DTid.x;
    const int y = DTid.y;

    if (x < ubo.width && y < ubo.height)
    {
        const float value = input[x + y * ubo.width];
        const int index = (x / 2 + 1) + (y + 1) * ubo.redBlackWidth;
        if (y % 2 == 0)
        {
            if (x % 2 == 0)
            {
                redOutput[index] = value;
            }
            else
            {
                blackOutput[index] = value;
            }
        }
        else
        {
            if (x % 2 == 0)
            {
                blackOutput[index] = value;
            }
            else
            {
                redOutput[index] = value;
            }
        }
    }
}
