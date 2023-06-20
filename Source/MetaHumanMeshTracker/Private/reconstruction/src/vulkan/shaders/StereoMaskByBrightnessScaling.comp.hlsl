// Copyright Epic Games, Inc. All Rights Reserved.

#include "StereoMaskByBrightnessScaling.comp.h"

[[vk::push_constant]] ConstantBuffer<StereoMaskByBrightnessScalingPushConstants> ubo;

[[vk::binding(0)]] StructuredBuffer<float> srcImage;
[[vk::binding(1)]] StructuredBuffer<float> targetImage;
[[vk::binding(2)]] RWStructuredBuffer<float> disparityImage;

#include "StereoHelpers.hlsl"

[numthreads(StereoMaskByBrightnessScalingComputeThreadSizeX, StereoMaskByBrightnessScalingComputeThreadSizeY, 1)]
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

    if (valueSrc > 0.0) {
        const float scale = valueTarget/valueSrc;
        if (abs(ubo.mainScale - scale) > ubo.scaleDifferenceThreshold) {
            disparityImage[y * w + x] = 0;
        }
    }
}
