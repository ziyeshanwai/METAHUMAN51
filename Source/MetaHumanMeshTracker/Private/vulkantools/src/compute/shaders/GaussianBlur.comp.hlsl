// Copyright Epic Games, Inc. All Rights Reserved.

#include "GaussianBlur.comp.h"

[[vk::push_constant]] ConstantBuffer<GaussianBlurPushConstants> ubo;

// StructuredBuffer implementation
[[vk::binding(0)]] StructuredBuffer<float> inputImage;
[[vk::binding(1)]] RWStructuredBuffer<float> outputImage;
[[vk::binding(2)]] StructuredBuffer<float> inputWeights;

[numthreads(GaussianBlurComputeThreadSizeX, GaussianBlurComputeThreadSizeY, 1)]
void main(uint3 DTid : SV_DispatchThreadID) {

    const int x = DTid.x;
    const int y = DTid.y;

    const int w = ubo.width;
    const int h = ubo.height;

    if(x >= w || y >= h)
        return;

    float total = 0;
    for (int i = -ubo.kernelSize2; i <= ubo.kernelSize2; ++i) {
        const int xi = clamp(x + i * ubo.xStep, 0, w - 1);
        const int yi = clamp(y + i * ubo.yStep, 0, h - 1);
        const float weight = inputWeights[ubo.kernelSize2 + i];
        total += inputImage[yi * w + xi] * weight;
    }
    outputImage[y * w + x] = total;
}
