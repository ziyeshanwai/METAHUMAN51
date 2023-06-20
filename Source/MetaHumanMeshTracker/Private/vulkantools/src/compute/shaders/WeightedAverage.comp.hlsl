// Copyright Epic Games, Inc. All Rights Reserved.

#include "WeightedAverage.comp.h"

[[vk::push_constant]] ConstantBuffer<WeightedAveragePushConstants> ubo;

[[vk::binding(0)]] StructuredBuffer<float> inputA;
[[vk::binding(1)]] StructuredBuffer<float> inputB;
[[vk::binding(2)]] RWStructuredBuffer<float> output;

[numthreads(WeightedAverageComputeThreadSizeX, WeightedAverageComputeThreadSizeY, 1)]
void main(const uint3 DTid : SV_DispatchThreadID)
{
    const int x = DTid.x;
    const int y = DTid.y;

    if (x < ubo.width && y < ubo.height)
    {
        const float valueA = inputA[x + y * ubo.width];
        const float valueB = inputB[x + y * ubo.width];
        output[x + y * ubo.width] = ubo.weightA * valueA + ubo.weightB * valueB + ubo.delta;
    }
}
