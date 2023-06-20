// Copyright Epic Games, Inc. All Rights Reserved.

inline float PhotometricWeight(float v1, float v2, float v3)
{
  if (v2 < v1 && v2 < v3) {
    return 0.5f * (v1 + v3 - 2.0f * v2);
  } else if (v2 < v1) {
    return (v2 - v3);
  } else if (v2 < v3) {
    return (v2 - v1);
  } else {
    return 0.0f;
  }
}

inline float QuadraticInterpolation(float v1, float v2, float v3)
{
  if (v2 < v1 && v2 < v3) {
    return 0.5f * (v1 - v3) / (v1 + v3 - 2.0f * v2);
  } else if (v2 < v1) {
    return 0.5f;
  } else if (v2 < v3) {
    return -0.5f;
  } else {
    return 0.0f;
  }
}


inline float NormalizedCrossCorrelationInteger(const int KERNEL_RADIUS, StructuredBuffer<float> textureLeft, StructuredBuffer<float> textureRight, int w, int x, int y, int targetX)
{
    float ab = 0;
    float aa = 0;
    float bb = 0;
    for (int dy = -KERNEL_RADIUS; dy <= KERNEL_RADIUS; dy++) {
        for (int dx = -KERNEL_RADIUS; dx <= KERNEL_RADIUS; dx++) {
            const float a = textureLeft[(y + dy) * w + (x + dx)];
            const float b = textureRight[(y + dy) * w + (targetX + dx)];
            ab += a * b;
            aa += a * a;
            bb += b * b;
        }
    }
    return (1.0f - ab * rsqrt(aa * bb)) * 0.5f;
}


inline float bilinearInterpolateX(StructuredBuffer<float> buffer, int width, float x, int y)
{
    int xi = int(floor(x));
    float w2 = x - xi;
    return (1.f - w2) * buffer[y * width + xi] + w2 * buffer[y * width + xi + 1];
}


inline float NormalizedCrossCorrelation(StructuredBuffer<float> bufferLeft, StructuredBuffer<float> bufferRight, int width, int x, int y, float targetX)
{
    float ab = 0;
    float aa = 0;
    float bb = 0;
    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
            const float a = bufferLeft[(y + dy) * width + (x + dx)];
            const float b = bilinearInterpolateX(bufferRight, width, targetX + dx, y + dy);
            ab += a * b;
            aa += a * a;
            bb += b * b;
        }
    }
    return (1.0f - ab * rsqrt(aa * bb)) * 0.5f;
}
