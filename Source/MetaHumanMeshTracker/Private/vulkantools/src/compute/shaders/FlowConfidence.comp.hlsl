// Copyright Epic Games, Inc. All Rights Reserved.

#include "FlowConfidence.comp.h"

[[vk::push_constant]] ConstantBuffer<FlowConfidencePushConstants> ubo;

[[vk::binding(0)]] StructuredBuffer<float> flowU;
[[vk::binding(1)]] StructuredBuffer<float> flowV;
[[vk::binding(2)]] StructuredBuffer<float> invFlowU;
[[vk::binding(3)]] StructuredBuffer<float> invFlowV;
[[vk::binding(4)]] RWStructuredBuffer<float> outConf;

#include "HelperFunctions.h"

[numthreads(FlowConfidenceThreadSizeX, FlowConfidenceThreadSizeY, 1)]
void main(const uint3 DTid : SV_DispatchThreadID)
{
    const int x = DTid.x;
    const int y = DTid.y;

    if (x < ubo.width && y < ubo.height)
    {
        const int index = x + y * ubo.width;

        const float du = flowU[index];
        const float dv = flowV[index];

        const float xf = x + du;
        const float yf = y + dv;

        const int w = ubo.width - 1;
        const int h = ubo.height - 1;

        float dinvu;
        float dinvv;

        switch (ubo.interpolationMode)
        {
            case INTERP_MODE_NEAREST:
                dinvu = NearestInterpolate2(invFlowU, float2(xf, yf), int2(w, h));
                dinvv = NearestInterpolate2(invFlowV, float2(xf, yf), int2(w, h));
                break;
            case INTERP_MODE_BILINEAR:
                dinvu = BilinearInterpolate(invFlowU, xf, yf, w, h);
                dinvv = BilinearInterpolate(invFlowV, xf, yf, w, h);
                break;
            case INTERP_MODE_CUBIC:
                dinvu = CubicInterpolate(invFlowU, xf, yf, w, h);
                dinvu = CubicInterpolate(invFlowV, xf, yf, w, h);
                break;
            default:
                abort();
        }

        const float nu = du + dinvu;
        const float nv = dv + dinvv;
        const float flowNorm = sqrt(nu * nu + nv * nv);
        outConf[index] = max(0.0f, (1.0f - flowNorm));
    }
}
