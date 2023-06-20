// Copyright Epic Games, Inc. All Rights Reserved.

#include "CalculateSmoothnessTerm.comp.h"

[[vk::push_constant]] ConstantBuffer<CalculateSmoothnessTermPushConstants> ubo;

[[vk::binding(0)]] StructuredBuffer<float> u_const_curr; // u offset which stays constant
[[vk::binding(1)]] StructuredBuffer<float> v_const_curr; // v offset which stays constant
[[vk::binding(2)]] StructuredBuffer<float> u_const_other;
[[vk::binding(3)]] StructuredBuffer<float> v_const_other;

[[vk::binding(4)]] RWStructuredBuffer<float> weights_curr;
[[vk::binding(5)]] RWStructuredBuffer<float> A11_curr;
[[vk::binding(6)]] RWStructuredBuffer<float> b1_curr;
[[vk::binding(7)]] RWStructuredBuffer<float> A22_curr;
[[vk::binding(8)]] RWStructuredBuffer<float> b2_curr;
[[vk::binding(9)]] RWStructuredBuffer<float> A11_other;
[[vk::binding(10)]] RWStructuredBuffer<float> b1_other;
[[vk::binding(11)]] RWStructuredBuffer<float> A22_other;
[[vk::binding(12)]] RWStructuredBuffer<float> b2_other;

[[vk::binding(13)]] RWStructuredBuffer<float> u_update_curr; // u offset which is constantly updated (u + du)
[[vk::binding(14)]] RWStructuredBuffer<float> v_update_curr; // v offset which is constantly updated (v + dv)
[[vk::binding(15)]] RWStructuredBuffer<float> u_update_other;
[[vk::binding(16)]] RWStructuredBuffer<float> v_update_other;


[numthreads(CalculateSmoothnessTermThreadSizeX, CalculateSmoothnessTermThreadSizeY, 1)]
void main(const uint3 DTid : SV_DispatchThreadID)
{
    const int x = DTid.x;
    const int y = DTid.y;

    const bool isEven = ((y % 2) == 0);
    const bool isOdd = !isEven;

    if (ubo.direction == SMOOTHNESS_TERM_DIRECTION_VERTICAL)
    {
        //! omit the last row in the vertical pass as there is no smoothness term there
        if (x < (isEven ? ubo.evenLen : ubo.oddLen) && y < ubo.height - 1)
        {
            const int currIndex = (x + 1) + (y + 1) * ubo.width;
            const int otherIndex = (x + 1) + (y + 2) * ubo.width;

            const float weight = weights_curr[currIndex];
            const float u_const = u_const_curr[currIndex];
            const float u_const_next_row = u_const_other[otherIndex];
            const float v_const = v_const_curr[currIndex];
            const float v_const_next_row = v_const_other[otherIndex];

            const float uy = weight * (u_const_next_row - u_const);
            const float vy = weight * (v_const_next_row - v_const);

            b1_curr[currIndex] += uy;
            A11_curr[currIndex] += weight;

            b2_curr[currIndex] += vy;
            A22_curr[currIndex] += weight;

            b1_other[otherIndex] -= uy;
            A11_other[otherIndex] += weight;

            b2_other[otherIndex] -= vy;
            A22_other[otherIndex] += weight;
        }
    }
    else if (ubo.direction == SMOOTHNESS_TERM_DIRECTION_HORIZONTAL)
    {
        if (x < (isEven ? ubo.evenLen : ubo.oddLen) && y < ubo.height)
        {
            const int stepX_other = ((ubo.isRedPass ^ isOdd) ? 0 : 1);

            const int currIndex = (x + 1) + (y + 1) * ubo.width;
            const int otherIndex = (x + 1 + stepX_other) + (y + 1) * ubo.width;
            const int nextRowIndex = (x + 1) + (y + 2) * ubo.width;

            const float u_const = u_const_curr[currIndex];
            const float u_const_next = u_const_other[otherIndex];
            const float v_const = v_const_curr[currIndex];
            const float v_const_next = v_const_other[otherIndex];

            const float u_update = u_update_curr[currIndex];
            const float u_update_next = u_update_other[otherIndex];
            const float u_update_next_row = u_update_other[nextRowIndex];

            const float v_update = v_update_curr[currIndex];
            const float v_update_next = v_update_other[otherIndex];
            const float v_update_next_row = v_update_other[nextRowIndex];

            // Gradients for the flow on the current fixed-point iteration:
            float ux = u_update_next - u_update;
            float vx = v_update_next - v_update;
            float uy = u_update_next_row - u_update;
            float vy = v_update_next_row - v_update;

            // Weight of the smoothness term in the current fixed-point iteration
            const float weight = ubo.params.alpha2 / sqrt(ux * ux + vx * vx + uy * uy + vy * vy + ubo.params.epsilonSquared);

            // Gradients for the initial raw flow multiplied by weight
            ux = weight * (u_const_next - u_const);
            vx = weight * (v_const_next - v_const);

            // we still need to write the weight for the vertical pass
            weights_curr[currIndex] = weight;

            bool touches_right_border;
            if (isEven)
            {
                touches_right_border = (ubo.isRedPass && ubo.evenLen != ubo.oddLen && x == (ubo.evenLen - 1)) ||
                                       (!ubo.isRedPass && ubo.evenLen == ubo.oddLen && x == (ubo.evenLen - 1));
            }
            else
            {
                touches_right_border = (ubo.isRedPass && ubo.evenLen == ubo.oddLen && x == (ubo.oddLen - 1)) ||
                                       (!ubo.isRedPass && ubo.evenLen != ubo.oddLen && x == (ubo.oddLen - 1));
            }

            if (!touches_right_border)
            {
                b1_curr[currIndex] += ux;
                A11_curr[currIndex] += weight;

                b2_curr[currIndex] += vx;
                A22_curr[currIndex] += weight;

                b1_other[otherIndex] -= ux;
                A11_other[otherIndex] += weight;

                b2_other[otherIndex] -= vx;
                A22_other[otherIndex] += weight;
            }
        }
    }
    else
    {
        abort();
    }
}
