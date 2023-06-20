// Copyright Epic Games, Inc. All Rights Reserved.

#include <reconstruction/vulkan/VulkanDisparityToDepth.h>
#include <nls/vulkan/common/VulkanShaderModule.h>

#include <carbon/Common.h>

#include "shaders/embed_DisparityToDepth.comp.spv.h"
// include shader for push constant definition and size of thread
#include "shaders/DisparityToDepth.comp.h"
#include "shaders/DisparityToDepth.comp.refl.h"

#include <cmath>

namespace epic::nls {


bool VulkanDisparityToDepth::Init()
{
    return Kernel::Init< DisparityToDepthReflection, DisparityToDepthPushConstants >(SPV_DisparityToDepth_comp_spv, sizeof(SPV_DisparityToDepth_comp_spv));
}

void VulkanDisparityToDepth::DisparityToDepth(
    std::shared_ptr<VulkanComputeBuffer> inputBuffer,
    std::shared_ptr<VulkanComputeBuffer> outputBuffer,
    float f,
    float b,
    float offset,
    float scaleDisparity,
    VulkanComputeContext::HBatch batch
)
{
    CARBON_PRECONDITION(inputBuffer, "input buffer must be valid");
    CARBON_PRECONDITION(outputBuffer, "output buffer must be valid");
    CARBON_PRECONDITION(inputBuffer->Width() == outputBuffer->Width(), "input and output buffer must have same width");
    CARBON_PRECONDITION(inputBuffer->Height() == outputBuffer->Height(), "input and output buffer must have same height");

    const int w = inputBuffer->Width();
    const int h = inputBuffer->Height();
    DisparityToDepthPushConstants ubo = {w, h, f * b, offset, scaleDisparity };

    DisparityToDepthReflection::DescSet0_Update dsu(inputBuffer->ManagedBuffer()->Buffer(), outputBuffer->ManagedBuffer()->Buffer());

    Run(batch,
        {
            (uint32_t)std::ceil(w / float(DisparityToDepthComputeThreadSizeX)), 
            (uint32_t)std::ceil(h / float(DisparityToDepthComputeThreadSizeY)), 
            1
        },
        dsu.writes, dsu.nWrites,
        &ubo);
}

} // namespace epic::nls
