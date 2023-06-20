// Copyright Epic Games, Inc. All Rights Reserved.

#include <reconstruction/vulkan/VulkanDepthToDepthAndNormal.h>
#include <nls/vulkan/common/VulkanShaderModule.h>

#include <carbon/Common.h>

#include "shaders/embed_DepthToDepthAndNormal.comp.spv.h"
// include shader for push constant definition and size of thread
#include "shaders/DepthToDepthAndNormal.comp.h"
#include "shaders/DepthToDepthAndNormal.comp.refl.h"

#include <cmath>

namespace epic::nls {

bool VulkanDepthToDepthAndNormal::Init()
{
    return Kernel::Init< DepthToDepthAndNormalReflection, DepthToDepthAndNormalPushConstants >(SPV_DepthToDepthAndNormal_comp_spv, sizeof(SPV_DepthToDepthAndNormal_comp_spv));
}

void VulkanDepthToDepthAndNormal::DepthToDepthAndNormal(
    std::shared_ptr<VulkanComputeBuffer> inputBuffer,
    std::shared_ptr<VulkanComputeBuffer> outputBuffer,
    float fx,
    float fy,
    float skew,
    float cx,
    float cy,
    VulkanComputeContext::HBatch batch
)
{
    CARBON_PRECONDITION(inputBuffer, "input buffer must be valid");
    CARBON_PRECONDITION(outputBuffer, "output buffer must be valid");
    CARBON_PRECONDITION(inputBuffer->Width() * 4== outputBuffer->Width(), "output buffer must have 4 times the width of the input buffer");
    CARBON_PRECONDITION(inputBuffer->Height() == outputBuffer->Height(), "input and output buffer must have same height");

    const int w = inputBuffer->Width();
    const int h = inputBuffer->Height();
    DepthToDepthAndNormalPushConstants ubo = {w, h, fx, fy, skew, cx, cy };

    DepthToDepthAndNormalReflection::DescSet0_Update    dsu(inputBuffer->ManagedBuffer()->Buffer(), outputBuffer->ManagedBuffer()->Buffer());

    Run(batch,
        {
            (uint32_t)std::ceil(w / float(DepthToDepthAndNormalComputeThreadSizeX)), 
            (uint32_t)std::ceil(h / float(DepthToDepthAndNormalComputeThreadSizeY)), 
            1
        },
        dsu.writes, dsu.nWrites,
        &ubo);
}

} // namespace epic::nls
