// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/math/Math.h>

namespace epic::nls {

enum class InterpolationMethod {
    NEAREST,
    LINEAR,
    CUBIC
};

//! nearest texture interpolation
template <typename TYPE>
inline TYPE nearestInterpolate(const TYPE* texture, float x, float y, int maxX, int maxY)
{
    int xi1 = int(round(x));
    int yi1 = int(round(y));
    xi1 = clamp(xi1, 0, maxX);
    yi1 = clamp(yi1, 0, maxY);
    return texture[yi1 * (maxX + 1) + xi1];
}

//! bilinear texture interpolation
template <typename TYPE>
inline TYPE bilinearInterpolate(const TYPE* texture, float x, float y, int maxX, int maxY)
{
    int xi1 = int(floor(x));
    int yi1 = int(floor(y));
    int xi2 = xi1 + 1;
    int yi2 = yi1 + 1;
    float wx2 = x - xi1;
    float wy2 = y - yi1;
    xi1 = clamp(xi1, 0, maxX);
    xi2 = clamp(xi2, 0, maxX);
    yi1 = clamp(yi1, 0, maxY);
    yi2 = clamp(yi2, 0, maxY);
    return (1.f - wx2) * (1.f - wy2) * texture[yi1 * (maxX + 1) + xi1] +
           (      wx2) * (1.f - wy2) * texture[yi1 * (maxX + 1) + xi2] +
           (1.f - wx2) * (      wy2) * texture[yi2 * (maxX + 1) + xi1] +
           (      wx2) * (      wy2) * texture[yi2 * (maxX + 1) + xi2];
}

//! cubic interpolation based on implementation in OpenCV
template <typename TYPE>
inline TYPE cubicValue(float t, TYPE fm, TYPE f0, TYPE f1, TYPE f2)
{
    const float A = -0.75f;
    const float coeff0 = ((A * (t + 1) - 5 * A) * (t + 1) + 8 * A) * (t + 1) - 4 * A;
    const float coeff1 = ((A + 2)*t - (A + 3)) * t * t + 1;
    const float coeff2 = ((A + 2) * (1 - t)  - (A + 3)) * (1 - t) * (1 - t) + 1;
    const float coeff3 = 1.f - coeff0 - coeff1 - coeff2;
    return coeff0 * fm + coeff1 * f0 + coeff2 * f1 + coeff3 * f2;
}

//! cubic texture interpolation
template <typename TYPE>
inline TYPE cubicInterpolate(const TYPE* texture, float x, float y, int maxX, int maxY)
{
    int xi1 = int(floor(x));
    int yi1 = int(floor(y));
    int xi2 = xi1 + 1;
    int yi2 = yi1 + 1;
    const float tx1 = (x - xi1);
    const float ty1 = (y - yi1);
    const int xi0 = clamp(xi1 - 1, 0, maxX);
    const int xi3 = clamp(xi2 + 1, 0, maxX);
    xi1 = clamp(xi1, 0, maxX);
    xi2 = clamp(xi2, 0, maxX);

    const int yi0 = clamp(yi1 - 1, 0, maxY);
    const int yi3 = clamp(yi2 + 1, 0, maxY);
    yi1 = clamp(yi1, 0, maxY);
    yi2 = clamp(yi2, 0, maxY);
    const TYPE bm = cubicValue(tx1, texture[yi0 * (maxX + 1) + xi0], texture[yi0 * (maxX + 1) + xi1], texture[yi0 * (maxX + 1) + xi2], texture[yi0 * (maxX + 1) + xi3]);
    const TYPE b0 = cubicValue(tx1, texture[yi1 * (maxX + 1) + xi0], texture[yi1 * (maxX + 1) + xi1], texture[yi1 * (maxX + 1) + xi2], texture[yi1 * (maxX + 1) + xi3]);
    const TYPE b1 = cubicValue(tx1, texture[yi2 * (maxX + 1) + xi0], texture[yi2 * (maxX + 1) + xi1], texture[yi2 * (maxX + 1) + xi2], texture[yi2 * (maxX + 1) + xi3]);
    const TYPE b2 = cubicValue(tx1, texture[yi3 * (maxX + 1) + xi0], texture[yi3 * (maxX + 1) + xi1], texture[yi3 * (maxX + 1) + xi2], texture[yi3 * (maxX + 1) + xi3]);
    return cubicValue(ty1, bm, b0, b1, b2);
}

} // namespace epic::nls