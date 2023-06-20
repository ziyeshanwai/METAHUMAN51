// Copyright Epic Games, Inc. All Rights Reserved.

#include "DepthToDepthAndNormal.comp.h"

[[vk::push_constant]] ConstantBuffer<DepthToDepthAndNormalPushConstants> ubo;

[[vk::binding(0)]] StructuredBuffer<float> inputImage;
[[vk::binding(1)]] RWStructuredBuffer<float4> outputImage;

float3 pixToPosition(float px, float py, float depth, float fx, float fy, float skew, float cx, float cy)
{
    const float y = (py - cy) / fy;
    const float x = (px - cx - y * skew) / fx;
    return float3(x * depth, y * depth, depth);
}

[numthreads(DepthToDepthAndNormalComputeThreadSizeX, DepthToDepthAndNormalComputeThreadSizeY, 1)]
void main(uint3 DTid : SV_DispatchThreadID) {

    const int x = DTid.x;
    const int y = DTid.y;

    const int w = ubo.width;
    const int h = ubo.height;

    if(x >= w || y >= h)
        return;

    const float fx = ubo.fx;
    const float fy = ubo.fy;
    const float skew = ubo.skew;
    const float cx = ubo.cx;
    const float cy = ubo.cy;

    float4 depthAndNormal = float4(0,0,0,0);
    if (x > 0 && x < w - 1 && y > 0 && y < h - 1) {
        const float depth = inputImage[y * w + x];
        if (depth > 0) {
            const float px = x + 0.5f;
            const float py = y + 0.5f;
            const float3 pos = pixToPosition(px, py, depth, fx, fy, skew, cx, cy);
            const float depthXM = inputImage[y * w + x - 1];
            const float3 posXM = pixToPosition(px - 1, py, depthXM, fx, fy, skew, cx, cy);
            const float depthXP = inputImage[y * w + x + 1];
            const float3 posXP = pixToPosition(px + 1, py, depthXP, fx, fy, skew, cx, cy);
            const float depthYM = inputImage[(y - 1) * w + x];
            const float3 posYM = pixToPosition(px, py - 1, depthYM, fx, fy, skew, cx, cy);
            const float depthYP = inputImage[(y + 1) * w + x];
            const float3 posYP = pixToPosition(px, py + 1, depthYP, fx, fy, skew, cx, cy);

            float3 normal = float3(0,0,0);
            if (depthXP > 0 && depthYM > 0) {
                normal += cross(posXP - pos, posYM - pos);
            }
            if (depthYM > 0 && depthXM > 0) {
                normal += cross(posYM - pos, posXM - pos);
            }
            if (depthXM > 0 && depthYP > 0) {
                normal += cross(posXM - pos, posYP - pos);
            }
            if (depthYP > 0 && depthXP > 0) {
                normal += cross(posYP - pos, posXP - pos);
            }

            const float len = dot(normal, normal);
            if (len > 0) {
                depthAndNormal.x = depth;
                normal = rsqrt(len) * normal;
                depthAndNormal.y = normal.x;
                depthAndNormal.z = normal.y;
                depthAndNormal.w = normal.z;
            }
        }
    }

    outputImage[y * w + x] = depthAndNormal;
}
