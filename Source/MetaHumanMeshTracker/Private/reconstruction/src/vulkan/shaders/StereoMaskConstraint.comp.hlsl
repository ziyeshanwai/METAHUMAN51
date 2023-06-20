// Copyright Epic Games, Inc. All Rights Reserved.

#include "StereoMaskConstraint.comp.h"

[[vk::push_constant]] ConstantBuffer<StereoMaskConstraintPushConstants> ubo;

[[vk::binding(0)]] RWStructuredBuffer<float> disparityImage;
[[vk::binding(1)]] StructuredBuffer<float> leftMask;
[[vk::binding(2)]] StructuredBuffer<float> rightMask;


[numthreads(StereoMaskConstraintComputeThreadSizeX, StereoMaskConstraintComputeThreadSizeY, 1)]
void main(uint3 DTid : SV_DispatchThreadID) {

    const int x = DTid.x;
    const int y = DTid.y;

    const int w = ubo.width;
    const int h = ubo.height;

    if(x >= w || y >= h)
        return;

    const int tw = ubo.targetWidth;

    const float disparity = disparityImage[y * w + x];

    if (disparity != 0) {
        const float mask = leftMask[y * w + x];

        const float px = x + disparity;
        int xi1 = int(floor(px));
        int xi2 = xi1 + 1;
        const float wx2 = px - xi1;
        xi1 = clamp(xi1, 0, tw - 1);
        xi2 = clamp(xi2, 0, tw - 1);

        const float maskOther = (1.f - wx2) * rightMask[y * tw + xi1] +
                                (      wx2) * rightMask[y * tw + xi2];

        if (mask < 0.99f || maskOther < 0.99f) {
            disparityImage[y * w + x] = 0;
        }
    }
}
