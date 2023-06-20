// Copyright Epic Games, Inc. All Rights Reserved.

#include "CalculateDataTerm.comp.h"

[[vk::push_constant]] ConstantBuffer<CalculateDataTermPushConstants> ubo;

[[vk::binding(0)]] StructuredBuffer<float> Ix;
[[vk::binding(1)]] StructuredBuffer<float> Iy;
[[vk::binding(2)]] StructuredBuffer<float> Iz;
[[vk::binding(3)]] StructuredBuffer<float> Ixx;
[[vk::binding(4)]] StructuredBuffer<float> Ixy;
[[vk::binding(5)]] StructuredBuffer<float> Iyy;
[[vk::binding(6)]] StructuredBuffer<float> Ixz;
[[vk::binding(7)]] StructuredBuffer<float> Iyz;

[[vk::binding(8)]] StructuredBuffer<float> dW_u;
[[vk::binding(9)]] StructuredBuffer<float> dW_v;

[[vk::binding(10)]] RWStructuredBuffer<float> A11;
[[vk::binding(11)]] RWStructuredBuffer<float> A12;
[[vk::binding(12)]] RWStructuredBuffer<float> A22;
[[vk::binding(13)]] RWStructuredBuffer<float> b1;
[[vk::binding(14)]] RWStructuredBuffer<float> b2;



[numthreads(CalculateDataTermThreadSizeX, CalculateDataTermThreadSizeY, 1)]
void main(const uint3 DTid : SV_DispatchThreadID)
{
    const int x = DTid.x;
    const int y = DTid.y;

    if (x < ((y % 2) ? ubo.oddLen : ubo.evenLen) && y < ubo.height)
    {
        const int Index = (x + 1) + (y + 1) * ubo.width;
        const float vx = Ix[Index];
        const float vy = Iy[Index];
        const float vz = Iz[Index];
        const float vxx = Ixx[Index];
        const float vxy = Ixy[Index];
        const float vyy = Iyy[Index];
        const float vxz = Ixz[Index];
        const float vyz = Iyz[Index];
        const float du = dW_u[Index];
        const float dv = dW_v[Index];

        // Step 1: Compute color constancy terms
        // Normalization Factor
        float derivNorm = vx * vx + vy * vy + ubo.params.zetaSquared;
        // Color constancy penalty (computed by Taylor expansion):
        const float Ik1z = vz + vx * du + vy * dv;
        // Weight the color constancy term in the current fixed-point iteration divided by derivNorm
        float weight = (ubo.params.delta2 / sqrt(Ik1z * Ik1z / derivNorm + ubo.params.epsilonSquared)) / derivNorm;
        // Add respective color constancy terms to the linear system coefficients
        float vA11 = weight * (vx * vx) + ubo.params.regularizationSquared;
        float vA12 = weight * (vx * vy);
        float vA22 = weight * (vy * vy) + ubo.params.regularizationSquared;
        float vb1 = -weight * (vz * vx);
        float vb2 = -weight * (vz * vy);

        // Step 2: Compute gradient constancy terms
        // Normalization factor for x gradient
        derivNorm = vxx * vxx + vxy * vxy + ubo.params.zetaSquared;
        // Normalization factor for y gradient
        const float derivNorm2 = vyy * vyy + vxy * vxy + ubo.params.zetaSquared;
        // Gradient constancy penalties (computed by Taylor expansion):
        const float Ik1zx = vxz + vxx * du + vxy * dv;
        const float Ik1zy = vyz + vxy * du + vyy * dv;
        // Weight of the gradient constancy term in the current fixed-point iteration:
        weight = ubo.params.gamma2 / sqrt(Ik1zx * Ik1zx / derivNorm + Ik1zy * Ik1zy / derivNorm2 + ubo.params.epsilonSquared);
        // Add respective gradient constancy components to the linear system coefficients:
        vA11 += weight * (vxx * vxx / derivNorm + vxy * vxy / derivNorm2);
        vA12 += weight * (vxx * vxy / derivNorm + vxy * vyy / derivNorm2);
        vA22 += weight * (vxy * vxy / derivNorm + vyy * vyy / derivNorm2);
        vb1 += -weight * (vxx * vxz / derivNorm + vxy * vyz / derivNorm2);
        vb2 += -weight * (vxy * vxz / derivNorm + vyy * vyz / derivNorm2);

        A11[Index] = vA11;
        A12[Index] = vA12;
        A22[Index] = vA22;
        b1[Index] = vb1;
        b2[Index] = vb2;
    }
}
