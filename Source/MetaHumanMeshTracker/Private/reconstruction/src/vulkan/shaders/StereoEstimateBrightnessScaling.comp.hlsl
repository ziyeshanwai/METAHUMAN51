// Copyright Epic Games, Inc. All Rights Reserved.

#include "StereoEstimateBrightnessScaling.comp.h"

[[vk::push_constant]] ConstantBuffer<StereoEstimateBrightnessScalingPushConstants> ubo;

[[vk::binding(0)]] StructuredBuffer<float> srcImage;
[[vk::binding(1)]] StructuredBuffer<float> targetImage;
[[vk::binding(2)]] StructuredBuffer<float> disparityImage;
[[vk::binding(3)]] RWStructuredBuffer<float> scalingImage;

#include "StereoHelpers.hlsl"

[numthreads(StereoEstimateBrightnessScalingComputeThreadSizeX, StereoEstimateBrightnessScalingComputeThreadSizeY, 1)]
void main(uint3 DTid : SV_DispatchThreadID) {

    const int x = DTid.x;
    const int y = DTid.y;

    const int w = ubo.width;
    const int h = ubo.height;

    if(x >= w || y >= h)
        return;

    const float valueSrc = srcImage[y * w + x];
    const float targetX = x + disparityImage[y * w + x];
    const float valueTarget = bilinearInterpolateX(targetImage, w, targetX, y);

    float scale = 0;
    if (valueSrc > 0.0001 && valueTarget > 0.0001) {
        scale = valueTarget/valueSrc;
    }
    scalingImage[y * w + x] = scale;
}
