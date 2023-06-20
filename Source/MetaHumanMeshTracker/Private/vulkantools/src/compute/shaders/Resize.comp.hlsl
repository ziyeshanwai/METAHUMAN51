// Copyright Epic Games, Inc. All Rights Reserved.

#include "Resize.comp.h"

[[vk::push_constant]] ConstantBuffer<ResizePushConstants> ubo;

[[vk::binding(0)]] StructuredBuffer<float> input;
[[vk::binding(1)]] RWStructuredBuffer<float> output;

#include "HelperFunctions.h"

[numthreads(ResizeComputeThreadSizeX, ResizeComputeThreadSizeY, 1)]
void main(const uint3 DTid : SV_DispatchThreadID)
{
    const int x = DTid.x;
    const int y = DTid.y;

    if (x < ubo.outputWidth && y < ubo.outputHeight)
    {
        const float px = ubo.scaleX * (x + 0.5f) - 0.5f;
        const float py = ubo.scaleY * (y + 0.5f) - 0.5f;

        float sample;
        switch (ubo.interpolationMode)
        {
            case INTERP_MODE_NEAREST:
                sample = NearestInterpolate2(input, float2(ubo.scaleX * x - 0.5f, ubo.scaleY * y - 0.5f), int2(ubo.maxX, ubo.maxY));
                break;
            case INTERP_MODE_BILINEAR:
                sample = BilinearInterpolate(input, px, py, ubo.maxX, ubo.maxY);
                break;
            case INTERP_MODE_CUBIC:
                sample = CubicInterpolate(input, px, py, ubo.maxX, ubo.maxY);
                break;
            default:
                abort();
        }

        output[x + y * ubo.outputWidth] = ubo.scale * sample;
    }
}
