// Copyright Epic Games, Inc. All Rights Reserved.

#include "SOR.comp.h"

[[vk::push_constant]] ConstantBuffer<SORPushConstants> ubo;

[[vk::binding(0)]] RWStructuredBuffer<float> du_curr;
[[vk::binding(1)]] RWStructuredBuffer<float> dv_curr;
[[vk::binding(2)]] StructuredBuffer<float> du_other;
[[vk::binding(3)]] StructuredBuffer<float> dv_other;
[[vk::binding(4)]] StructuredBuffer<float> weights_curr;
[[vk::binding(5)]] StructuredBuffer<float> weights_other;
[[vk::binding(6)]] StructuredBuffer<float> A11_curr;
[[vk::binding(7)]] StructuredBuffer<float> A12_curr;
[[vk::binding(8)]] StructuredBuffer<float> A22_curr;
[[vk::binding(9)]] StructuredBuffer<float> b1_curr;
[[vk::binding(10)]] StructuredBuffer<float> b2_curr;

[numthreads(SORThreadSizeX, SORThreadSizeY, 1)]
void main(const uint3 DTid : SV_DispatchThreadID)
{
    const int x = DTid.x;
    const int y = DTid.y;

    const bool isEven = (y % 2 == 0);
    const bool isOdd = !isEven;

    if (x > 0 && x < ((isEven ? ubo.evenLen : ubo.oddLen) + 1) && y < ubo.height)
    {
        const int stepX_other = ((ubo.isRedPass ^ isOdd) ? 0 : 1);
        const int x1 = x;
        const int y1 = y + 1;
        const float weight = weights_curr[x1 + y1 * ubo.width];
        const float weight_prev = weights_other[(x1 + stepX_other - 1) + y1 * ubo.width];
        const float weight_prev_row = weights_other[x1 + (y1 - 1) * ubo.width];

        const float du = du_curr[x1 + y1 * ubo.width];
        const float du_prev = du_other[(x1 + stepX_other - 1) + y1 * ubo.width];
        const float du_next = du_other[(x1 + stepX_other) + y1 * ubo.width];
        const float du_prev_row = du_other[x1 + (y1 - 1) * ubo.width];
        const float du_next_row = du_other[x1 + (y1 + 1) * ubo.width];

        const float dv = dv_curr[x1 + y1 * ubo.width];
        const float dv_prev = dv_other[(x1 + stepX_other - 1) + y1 * ubo.width];
        const float dv_next = dv_other[(x1 + stepX_other) + y1 * ubo.width];
        const float dv_prev_row = dv_other[x1 + (y1 - 1) * ubo.width];
        const float dv_next_row = dv_other[x1 + (y1 + 1) * ubo.width];

        const float sigmaU = weight_prev * du_prev + weight * du_next + weight_prev_row * du_prev_row + weight * du_next_row;
        const float sigmaV = weight_prev * dv_prev + weight * dv_next + weight_prev_row * dv_prev_row + weight * dv_next_row;

        const float index = x1 + y1 * ubo.width;
        const float A11 = A11_curr[index];
        const float A12 = A12_curr[index];
        const float A22 = A22_curr[index];
        const float b1 = b1_curr[index];
        const float b2 = b2_curr[index];

        // opencv solve: first du, then dv
        // const float du_new = du + omega * ((sigmaU + b1 - dv * A12) / A11 - du);
        // const float dv_new = dv + omega * ((sigmaV + b2 - du_new * A12) / A22 - dv);

        // alternative solve: solve both du and dv at the same time
        const float B1 = b1 + sigmaU;
        const float B2 = b2 + sigmaV;
        const float det = A11 * A22 - A12 * A12;
        const float du_new = (1.0f - ubo.omega) * du + ubo.omega * (A22 * B1 - A12 * B2) / det;
        const float dv_new = (1.0f - ubo.omega) * dv + ubo.omega * (A11 * B2 - A12 * B1) / det;

        du_curr[index] = du_new;
        dv_curr[index] = dv_new;
    }
}
