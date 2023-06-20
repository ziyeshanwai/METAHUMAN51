// Copyright Epic Games, Inc. All Rights Reserved.

float srgb2linear(float srgb)
{
    const float offset = 0.055f;
    if (srgb < 0.04045f)
    {
        return srgb / 12.92f;
    }
    else
    {
        return pow((srgb + offset) / (1.0f + offset), 2.4f);
    }
}

#define INTERP_MODE_NEAREST   0
#define INTERP_MODE_BILINEAR  1
#define INTERP_MODE_CUBIC     2

float NearestInterpolate(StructuredBuffer<float> input, float x, float y, int maxX, int maxY)
{
    int xi1 = int(round(x));
    int yi1 = int(round(y));
    xi1 = clamp(xi1, 0, maxX);
    yi1 = clamp(yi1, 0, maxY);
    return input[xi1 + yi1 * (maxX + 1)];
}

float NearestInterpolate2(StructuredBuffer<float> input, float2 P, int2 Max)
{
    const int2 PI = clamp(int2(round(P)), 0, Max);
    return input[PI.x + PI.y * (Max.x + 1)];
}

float BilinearInterpolate(StructuredBuffer<float> input, float x, float y, int maxX, int maxY)
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

    float p0 = input[xi1 + yi1 * (maxX + 1)];
    float p1 = input[xi2 + yi1 * (maxX + 1)];
    float p2 = input[xi1 + yi2 * (maxX + 1)];
    float p3 = input[xi2 + yi2 * (maxX + 1)];

    return (1.0f - wx2) * (1.0f - wy2) * p0 +
        (0.0f + wx2) * (1.0f - wy2) * p1 +
        (1.0f - wx2) * (0.0f + wy2) * p2 +
        (0.0f + wx2) * (0.0f + wy2) * p3;
}

float CubicValue(float t, float fm, float f0, float f1, float f2)
{
    const float A = -0.75f;
    const float coeff0 = ((A * (t + 1) - 5 * A) * (t + 1) + 8 * A) * (t + 1) - 4 * A;
    const float coeff1 = ((A + 2) * t - (A + 3)) * t * t + 1;
    const float coeff2 = ((A + 2) * (1 - t) - (A + 3)) * (1 - t) * (1 - t) + 1;
    const float coeff3 = 1.f - coeff0 - coeff1 - coeff2;
    return coeff0 * fm + coeff1 * f0 + coeff2 * f1 + coeff3 * f2;
}

float CubicInterpolate(StructuredBuffer<float> input, float x, float y, int maxX, int maxY)
{
    int xi1 = int(floor(x));
    int yi1 = int(floor(y));
    int xi2 = xi1 + 1;
    int yi2 = yi1 + 1;
    const float tx1 = x - xi1;
    const float ty1 = y - yi1;
    const int xi0 = clamp(xi1 - 1, 0, maxX);
    const int xi3 = clamp(xi2 + 1, 0, maxX);
    xi1 = clamp(xi1, 0, maxX);
    xi2 = clamp(xi2, 0, maxX);

    const int yi0 = clamp(yi1 - 1, 0, maxY);
    const int yi3 = clamp(yi2 + 1, 0, maxY);
    yi1 = clamp(yi1, 0, maxY);
    yi2 = clamp(yi2, 0, maxY);

    const float width = maxX + 1;
    const float p0 = input[xi0 + yi0 * width];
    const float p1 = input[xi1 + yi0 * width];
    const float p2 = input[xi2 + yi0 * width];
    const float p3 = input[xi3 + yi0 * width];

    const float p4 = input[xi0 + yi1 * width];
    const float p5 = input[xi1 + yi1 * width];
    const float p6 = input[xi2 + yi1 * width];
    const float p7 = input[xi3 + yi1 * width];

    const float p8 = input[xi0 + yi2 * width];
    const float p9 = input[xi1 + yi2 * width];
    const float p10 = input[xi2 + yi2 * width];
    const float p11 = input[xi3 + yi2 * width];

    const float p12 = input[xi0 + yi3 * width];
    const float p13 = input[xi1 + yi3 * width];
    const float p14 = input[xi2 + yi3 * width];
    const float p15 = input[xi3 + yi3 * width];

    const float bm = CubicValue(tx1, p0, p1, p2, p3);
    const float b0 = CubicValue(tx1, p4, p5, p6, p7);
    const float b1 = CubicValue(tx1, p8, p9, p10, p11);
    const float b2 = CubicValue(tx1, p12, p13, p14, p15);

    return CubicValue(ty1, bm, b0, b1, b2);
}
