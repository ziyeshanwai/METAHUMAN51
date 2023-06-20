// Copyright Epic Games, Inc. All Rights Reserved.

#include "Rectify.comp.h"

[[vk::push_constant]] ConstantBuffer<RectifyPushConstants> ubo;

[[vk::binding(0)]] StructuredBuffer<float> inputImage;
[[vk::binding(1)]] RWStructuredBuffer<float> outputImage;

// [[vk::binding(0, 0)]] RWTexture2D<float> inputImage;
// [[vk::binding(0, 1)]] RWTexture2D<float> outputImage;


float cubicValue(float t, float fm, float f0, float f1, float f2)
{
    const float A = -0.75f;
    const float coeff0 = ((A * (t + 1) - 5 * A) * (t + 1) + 8 * A) * (t + 1) - 4 * A;
    const float coeff1 = ((A + 2)*t - (A + 3)) * t * t + 1;
    const float coeff2 = ((A + 2) * (1 - t)  - (A + 3)) * (1 - t) * (1 - t) + 1;
    const float coeff3 = 1.f - coeff0 - coeff1 - coeff2;
    return coeff0 * fm + coeff1 * f0 + coeff2 * f1 + coeff3 * f2;
}


[numthreads(RectifyComputeThreadSizeX, RectifyComputeThreadSizeY, 1)]
void main(uint3 DTid : SV_DispatchThreadID) {

    const int x = DTid.x;
    const int y = DTid.y;

    if(x >= ubo.width || y >= ubo.height)
        return;

    const float xf = x + 0.5f;
    const float yf = y + 0.5f;
    float xmap = ubo.H[0] * xf + ubo.H[3] * yf + ubo.H[6];
    float ymap = ubo.H[1] * xf + ubo.H[4] * yf + ubo.H[7];
    float zmap = ubo.H[2] * xf + ubo.H[5] * yf + ubo.H[8];
    xmap /= zmap;
    ymap /= zmap;

    const float px = xmap - 0.5f;
    const float py = ymap - 0.5f;
    float value = 0;

    if (ubo.interpolationMethod == 0) {
        // nearest neighbor interpolation
        int xi1 = int(round(px));
        int yi1 = int(round(py));
        xi1 = clamp(xi1, 0, ubo.maxx);
        yi1 = clamp(yi1, 0, ubo.maxy);
        // value = inputImage[int2(xi1, yi1)];
        value = inputImage[yi1 * (ubo.maxx + 1) + xi1];
    } else if (ubo.interpolationMethod == 1) {
        // bilinear interpolation
        int xi1 = int(floor(px));
        int yi1 = int(floor(py));
        int xi2 = xi1 + 1;
        int yi2 = yi1 + 1;
        float wx2 = px - xi1;
        float wy2 = py - yi1;
        xi1 = clamp(xi1, 0, ubo.maxx);
        xi2 = clamp(xi2, 0, ubo.maxx);
        yi1 = clamp(yi1, 0, ubo.maxy);
        yi2 = clamp(yi2, 0, ubo.maxy);
        // value = (1.f - wx2) * (1.f - wy2) * inputImage[int2(xi1, yi1)] +
        //        (      wx2) * (1.f - wy2) * inputImage[int2(xi2, yi1)] +
        //        (1.f - wx2) * (      wy2) * inputImage[int2(xi1, yi2)] +
        //        (      wx2) * (      wy2) * inputImage[int2(xi2, yi2)];
        value = (1.f - wx2) * (1.f - wy2) * inputImage[yi1 * (ubo.maxx + 1) + xi1] +
               (      wx2) * (1.f - wy2) * inputImage[yi1 * (ubo.maxx + 1) + xi2] +
               (1.f - wx2) * (      wy2) * inputImage[yi2 * (ubo.maxx + 1) + xi1] +
               (      wx2) * (      wy2) * inputImage[yi2 * (ubo.maxx + 1) + xi2];
    } else if (ubo.interpolationMethod == 2) {
        // bicubic interpolation
        int xi1 = int(floor(px));
        int yi1 = int(floor(py));
        int xi2 = xi1 + 1;
        int yi2 = yi1 + 1;
        const float tx1 = (px - xi1);
        const float ty1 = (py - yi1);
        const int xi0 = clamp(xi1 - 1, 0, ubo.maxx);
        const int xi3 = clamp(xi2 + 1, 0, ubo.maxx);
        xi1 = clamp(xi1, 0, ubo.maxx);
        xi2 = clamp(xi2, 0, ubo.maxx);

        const int yi0 = clamp(yi1 - 1, 0, ubo.maxy);
        const int yi3 = clamp(yi2 + 1, 0, ubo.maxy);
        yi1 = clamp(yi1, 0, ubo.maxy);
        yi2 = clamp(yi2, 0, ubo.maxy);
        const int wi = ubo.maxx + 1;
        // const float bm = cubicValue(tx1, inputImage[int2(xi0, yi0)], inputImage[int2(xi1, yi0)], inputImage[int2(xi2, yi0)], inputImage[int2(xi3, yi0)]);
        // const float b0 = cubicValue(tx1, inputImage[int2(xi0, yi1)], inputImage[int2(xi1, yi1)], inputImage[int2(xi2, yi1)], inputImage[int2(xi3, yi1)]);
        // const float b1 = cubicValue(tx1, inputImage[int2(xi0, yi2)], inputImage[int2(xi1, yi2)], inputImage[int2(xi2, yi2)], inputImage[int2(xi3, yi2)]);
        // const float b2 = cubicValue(tx1, inputImage[int2(xi0, yi3)], inputImage[int2(xi1, yi3)], inputImage[int2(xi2, yi3)], inputImage[int2(xi3, yi3)]);
        const float bm = cubicValue(tx1, inputImage[yi0 * wi + xi0], inputImage[yi0 * wi + xi1], inputImage[yi0 * wi + xi2], inputImage[yi0 * wi + xi3]);
        const float b0 = cubicValue(tx1, inputImage[yi1 * wi + xi0], inputImage[yi1 * wi + xi1], inputImage[yi1 * wi + xi2], inputImage[yi1 * wi + xi3]);
        const float b1 = cubicValue(tx1, inputImage[yi2 * wi + xi0], inputImage[yi2 * wi + xi1], inputImage[yi2 * wi + xi2], inputImage[yi2 * wi + xi3]);
        const float b2 = cubicValue(tx1, inputImage[yi3 * wi + xi0], inputImage[yi3 * wi + xi1], inputImage[yi3 * wi + xi2], inputImage[yi3 * wi + xi3]);
        value = cubicValue(ty1, bm, b0, b1, b2);
    }

    // outputImage[int2(x,y)] = value;
    outputImage[y * ubo.width + x] = value;
}
