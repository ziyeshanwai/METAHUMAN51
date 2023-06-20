// Copyright Epic Games, Inc. All Rights Reserved.

#include "StereoUniquenessConstraint.comp.h"

[[vk::push_constant]] ConstantBuffer<StereoUniquenessConstraintPushConstants> ubo;

[[vk::binding(0)]] StructuredBuffer<float> inputDisparity;
[[vk::binding(1)]] StructuredBuffer<float> inputDisparityOther;
[[vk::binding(2)]] RWStructuredBuffer<float> outputDisparity;

[numthreads(StereoUniquenessConstraintComputeThreadSizeX, StereoUniquenessConstraintComputeThreadSizeY, 1)]
void main(uint3 DTid : SV_DispatchThreadID) {

    const int x = DTid.x;
    const int y = DTid.y;

    const int w = ubo.width;
    const int h = ubo.height;

    if(x >= w || y >= h)
        return;

    const float uniqunessThreshold = 1.5f;
    float disparity = inputDisparity[y * w + x];
    if (disparity != 0) {
        const int otherX = floor(x + disparity);
        if (otherX >= 0 && otherX < w - 1) {
            const float disparityOther1 = inputDisparityOther[y * w + otherX];
            const float disparityOther2 = inputDisparityOther[y * w + otherX + 1];
            if ((disparityOther1 != 0 && abs(disparityOther1 + disparity) > uniqunessThreshold) ||
                (disparityOther2 != 0 && abs(disparityOther2 + disparity) > uniqunessThreshold)
            ) {
                disparity = 0;
            }
        } else {
            disparity = 0;
        }
    }
    outputDisparity[y * w + x] = disparity;
}
