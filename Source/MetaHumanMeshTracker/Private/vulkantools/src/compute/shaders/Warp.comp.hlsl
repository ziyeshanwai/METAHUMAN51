// Copyright Epic Games, Inc. All Rights Reserved.

#include "Warp.comp.h"

[[vk::push_constant]] ConstantBuffer<WarpPushConstants> ubo;

[[vk::binding(0)]] StructuredBuffer<float> input;
[[vk::binding(1)]] StructuredBuffer<float> offsetX;
[[vk::binding(2)]] StructuredBuffer<float> offsetY;
[[vk::binding(3)]] RWStructuredBuffer<float> output;

#include "HelperFunctions.h"

[numthreads(WarpComputeThreadSizeX, WarpComputeThreadSizeY, 1)]
void main(const uint3 DTid : SV_DispatchThreadID)
{
    const int x = DTid.x;
    const int y = DTid.y;

    if (x < ubo.outputWidth && y < ubo.outputHeight)
    {
        const float width = ubo.maxX + 1;
        const float dx = offsetX[x + y * width];
        const float dy = offsetY[x + y * width];

        float sample;
        switch (ubo.interpolationMode)
        {
            case INTERP_MODE_NEAREST:
                sample = NearestInterpolate2(input, float2(x + dx, y + dy), int2(ubo.maxX, ubo.maxY));
                break;
            case INTERP_MODE_BILINEAR:
                sample = BilinearInterpolate(input, x + dx, y + dy, ubo.maxX, ubo.maxY);
                break;
            case INTERP_MODE_CUBIC:
                sample = CubicInterpolate(input, x + dx, y + dy, ubo.maxX, ubo.maxY);
                break;
            default:
                sample = 0.0f;
                break;
        }

        output[x + y * ubo.outputWidth] = sample;
    }
}
