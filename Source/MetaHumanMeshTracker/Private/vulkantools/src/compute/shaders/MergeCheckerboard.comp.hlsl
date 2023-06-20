// Copyright Epic Games, Inc. All Rights Reserved.

#include "MergeCheckerboard.comp.h"

[[vk::push_constant]] ConstantBuffer<MergeCheckerboardPushConstants> ubo;

[[vk::binding(0)]] StructuredBuffer<float> red;
[[vk::binding(1)]] StructuredBuffer<float> black;

[[vk::binding(2)]] RWStructuredBuffer<float> output;


[numthreads(MergeCheckerboardThreadSizeX, 1, 1)]
void main(const uint3 DTid : SV_DispatchThreadID)
{
    const int x = DTid.x;
    const int y = DTid.y;

    if (x < ubo.width && y < ubo.height)
    {
        const int readIndex = (x / 2 + 1) + (y + 1) * ubo.redBlackWidth;
        const int writeIndex = x + y * ubo.width;
        if (y % 2 == 0)
        {
            if (x % 2 == 0)
            {
                output[writeIndex] = red[readIndex];
            }
            else
            {
                output[writeIndex] = black[readIndex];
            }
        }
        else
        {
            if (x % 2 == 0)
            {
                output[writeIndex] = black[readIndex];
            }
            else
            {
                output[writeIndex] = red[readIndex];
            }
        }
    }
}
