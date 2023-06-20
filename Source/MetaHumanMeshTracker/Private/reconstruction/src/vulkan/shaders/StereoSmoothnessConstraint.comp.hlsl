// Copyright Epic Games, Inc. All Rights Reserved.

#include "StereoSmoothnessConstraint.comp.h"

[[vk::push_constant]] ConstantBuffer<StereoSmoothnessConstraintPushConstants> ubo;

[[vk::binding(0)]] StructuredBuffer<float> inputImage;
[[vk::binding(1)]] RWStructuredBuffer<float> outputImage;

[numthreads(StereoSmoothnessConstraintComputeThreadSizeX, StereoSmoothnessConstraintComputeThreadSizeY, 1)]
void main(uint3 DTid : SV_DispatchThreadID) {

    const int x = DTid.x;
    const int y = DTid.y;

    const int w = ubo.width;
    const int h = ubo.height;

    if(x >= w || y >= h)
        return;

    const int KERNEL_RADIUS = 1;

    float disparity = inputImage[y * w + x];
    if (disparity != 0) {
        if (x >= KERNEL_RADIUS && x < w - KERNEL_RADIUS && y >= KERNEL_RADIUS && y < h - KERNEL_RADIUS) {
            const float threshold = 0.5f;
            const int thresholdCount = int((2 * KERNEL_RADIUS + 1) * (2 * KERNEL_RADIUS + 1)  * threshold);
            int count = 0;
            for (int dy = -KERNEL_RADIUS; dy <=KERNEL_RADIUS; dy++) {
                for (int dx = -KERNEL_RADIUS; dx <=KERNEL_RADIUS; dx++) {
                    const float disparityOther = inputImage[(y + dy) * w + (x + dx)];
                    count += (abs(disparityOther - disparity) <= 1) ? 1 : 0;
                }
            }
            if (count <= thresholdCount) {
                disparity = 0;
            }
        }
    }
    outputImage[y * w + x] = disparity;
}
