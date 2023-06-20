// Copyright Epic Games, Inc. All Rights Reserved.

#include "UpdateRepeatedBorders.comp.h"

[[vk::push_constant]] ConstantBuffer<UpdateRepeatedBordersPushConstants> ubo;

[[vk::binding(0)]] RWStructuredBuffer<float> red;
[[vk::binding(1)]] RWStructuredBuffer<float> black;

[numthreads(UpdateRepeatedBordersThreadSizeX, 1, 1)]
void main(const uint3 DTid : SV_DispatchThreadID)
{
    const int x = DTid.x;
    const int y = DTid.y;

    if (ubo.borders == UPDATE_BORDERS_TOP_BOTTOM)
    {
        if (y == 0)
        {
            // copy top row
            if (x < ubo.width)
            {
                const int readIndex = x + 1 * ubo.width;
                const int writeIndex = x + 0 * ubo.width;
                red[writeIndex] = black[readIndex];
                black[writeIndex] = red[readIndex];
            }
        }
        else if (y == 1)
        {
            // copy bottom row
            if (x < ubo.width)
            {
                const int readIndex = x + (ubo.height - 2) * ubo.width;
                const int writeIndex = x + (ubo.height - 1) * ubo.width;
                red[writeIndex] = black[readIndex];
                black[writeIndex] = red[readIndex];
            }
        }
    }
    else if (ubo.borders == UPDATE_BORDERS_LEFT_RIGHT)
    {
        if (y == 0)
        {
            // copy left border
            if (x < ubo.height - 2)
            {
                const int row = x;
                const int readIndex = 1 + (row + 1) * ubo.width;
                const int writeIndex = 0 + (row + 1) * ubo.width;
                if (row % 2 == 0)
                {
                    black[writeIndex] = red[readIndex];
                    red[writeIndex] = red[readIndex]; // to make compatible with SplitCheckerboard
                }
                else
                {
                    red[writeIndex] = black[readIndex];
                    black[writeIndex] = black[readIndex]; // to make compatible with SplitCheckerboard
                }
            }
        }
        else if (y == 1)
        {
            // copy right border
            if (x < ubo.height - 2)
            {
                const int row = x;
                if (row % 2 == 0)
                {
                    if (ubo.numRedEven > ubo.numBlackEven)
                    {
                        // b_buf[black_even_len + 1] = r_buf[red_even_len];
                        const float value = red[ubo.numRedEven + (row + 1) * ubo.width];
                        black[(ubo.numBlackEven + 1) + (row + 1) * ubo.width] = value;
                        // b_buf[black_even_len + 2] = r_buf[black_even_len + 2] = value; // to make compatible with SplitCheckerboard
                        red[(ubo.numBlackEven + 2) + (row + 1) * ubo.width] = value;
                        black[(ubo.numBlackEven + 2) + (row + 1) * ubo.width] = value;
                    }
                    else
                    {
                        // r_buf[red_even_len + 1] = b_buf[black_even_len];
                        const float value = black[ubo.numBlackEven + (row + 1) * ubo.width];
                        red[(ubo.numRedEven + 1) + (row + 1) * ubo.width] = value;
                        // b_buf[red_even_len + 1] = value; // to make compatible with SplitCheckerboard
                        black[(ubo.numRedEven + 1) + (row + 1) * ubo.width] = value;
                    }
                }
                else
                {
                    if (ubo.numRedOdd < ubo.numBlackOdd)
                    {
                         // r_buf[red_odd_len + 1] = b_buf[black_odd_len];
                        const float value = black[ubo.numBlackOdd + (row + 1) * ubo.width];
                        red[(ubo.numRedOdd + 1) + (row + 1) * ubo.width] = value;
                        // b_buf[red_odd_len + 2] = r_buf[red_odd_len + 2] = value; // to make compatible with SplitCheckerboard
                        red[(ubo.numRedOdd + 2) + (row + 1) * ubo.width] = value;
                        black[(ubo.numRedOdd + 2) + (row + 1) * ubo.width] = value;
                    }
                    else
                    {
                        // b_buf[black_odd_len + 1] = r_buf[red_odd_len];
                        const float value = red[ubo.numRedOdd + (row + 1) * ubo.width];
                        black[(ubo.numBlackOdd + 1) + (row + 1) * ubo.width] = value;
                        // r_buf[black_odd_len + 1] = value; // to make compatible with SplitCheckerboard
                        red[(ubo.numBlackOdd + 1) + (row + 1) * ubo.width] = value;
                    }
                }
            }
        }
    }
}
