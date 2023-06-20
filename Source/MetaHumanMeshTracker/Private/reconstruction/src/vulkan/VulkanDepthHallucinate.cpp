// Copyright Epic Games, Inc. All Rights Reserved.

#include <reconstruction/vulkan/VulkanDepthHallucinate.h>
#include <nls/vulkan/common/VulkanShaderModule.h>

#include <carbon/Common.h>

#include "shaders/embed_DepthHallucinate.comp.spv.h"
// include shader for push constant definition and size of thread
#include "shaders/DepthHallucinate.comp.h"
#include "shaders/DepthHallucinate.comp.refl.h"

#include <cmath>

namespace epic::nls {


bool VulkanDepthHallucinate::Init()
{
    return Kernel::Init< DepthHallucinateReflection, DepthHallucinatePushConstants >(SPV_DepthHallucinate_comp_spv, sizeof(SPV_DepthHallucinate_comp_spv));
}

void VulkanDepthHallucinate::DepthHallucinate(
    std::shared_ptr<VulkanComputeBuffer> depthBuffer,
    std::shared_ptr<VulkanComputeBuffer> heightBuffer,
    std::shared_ptr<VulkanComputeBuffer> outputBuffer,
    float scale,
    VulkanComputeContext::HBatch batch
)
{
    CARBON_PRECONDITION(depthBuffer, "input buffer must be valid");
    CARBON_PRECONDITION(heightBuffer, "height buffer must be valid");
    CARBON_PRECONDITION(outputBuffer, "output buffer must be valid");
    CARBON_PRECONDITION(depthBuffer->Width() == outputBuffer->Width(), "input and output buffer must have same width");
    CARBON_PRECONDITION(depthBuffer->Height() == outputBuffer->Height(), "input and output buffer must have same height");
    CARBON_PRECONDITION(depthBuffer->Width() == heightBuffer->Width(), "input and height buffer must have same width");
    CARBON_PRECONDITION(depthBuffer->Height() == heightBuffer->Height(), "input and height buffer must have same height");

    const int w = depthBuffer->Width();
    const int h = depthBuffer->Height();
    DepthHallucinatePushConstants ubo = {w, h, scale };

    DepthHallucinateReflection::DescSet0_Update dsu(depthBuffer->ManagedBuffer()->Buffer(), heightBuffer->ManagedBuffer()->Buffer(), outputBuffer->ManagedBuffer()->Buffer());

    Run(batch,
        {
            (uint32_t)std::ceil(w / float(DepthHallucinateComputeThreadSizeX)),
            (uint32_t)std::ceil(h / float(DepthHallucinateComputeThreadSizeY)),
            1
        },
        dsu.writes, dsu.nWrites,
        &ubo);
}

} // namespace epic::nls
