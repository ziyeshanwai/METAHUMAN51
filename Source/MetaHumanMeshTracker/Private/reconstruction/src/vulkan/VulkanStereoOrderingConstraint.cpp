// Copyright Epic Games, Inc. All Rights Reserved.

#include <reconstruction/vulkan/VulkanStereoOrderingConstraint.h>
#include <nls/vulkan/common/VulkanShaderModule.h>
#include <nls/vulkan/common/VulkanDebugUtils.h>

#include <carbon/Common.h>

#include "shaders/embed_StereoOrderingConstraint.comp.spv.h"
// include shader for push constant definition and size of thread
#include "shaders/StereoOrderingConstraint.comp.h"
#include "shaders/StereoOrderingConstraint.comp.refl.h"

#include <cmath>

namespace epic::nls {

bool VulkanStereoOrderingConstraint::Init()
{
    return Kernel::Init< StereoOrderingConstraintReflection, StereoOrderingConstraintPushConstants >(SPV_StereoOrderingConstraint_comp_spv, sizeof(SPV_StereoOrderingConstraint_comp_spv));

//    VulkanObjectName::Set(m_vulkanComputeContext->Device()->Device(), m_vulkanComputeShaderPipeline->Pipeline(), "StereoOrderingConstraint:pipeline");
//    VulkanObjectName::Set(m_vulkanComputeContext->Device()->Device(), m_vulkanComputeShaderPipeline->PipelineLayout(), "StereoOrderingConstraint:pipelineLayout");
}

void VulkanStereoOrderingConstraint::ApplyOrderingConstraint(
    std::shared_ptr<VulkanComputeBuffer> inputDisparityBuffer,
    std::shared_ptr<VulkanComputeBuffer> outputDisparityBuffer,
    VulkanComputeContext::HBatch batch
)
{
    CARBON_PRECONDITION(inputDisparityBuffer, "input disparity buffer must be valid");
    CARBON_PRECONDITION(outputDisparityBuffer, "output disparity buffer must be valid");
    CARBON_PRECONDITION(inputDisparityBuffer->Width() == outputDisparityBuffer->Width(), "input and output disparity buffers must have same width");
    CARBON_PRECONDITION(inputDisparityBuffer->Height() == outputDisparityBuffer->Height(), "input and output disparity buffers must have same height");

    const int w = inputDisparityBuffer->Width();
    const int h = inputDisparityBuffer->Height();
    StereoOrderingConstraintPushConstants ubo = {w, h };

    StereoOrderingConstraintReflection::DescSet0_Update dsu(inputDisparityBuffer->ManagedBuffer()->Buffer(), outputDisparityBuffer->ManagedBuffer()->Buffer());

    Run(batch,
        {
            (uint32_t)std::ceil(w / float(StereoOrderingConstraintComputeThreadSizeX)),
            (uint32_t)std::ceil(h / float(StereoOrderingConstraintComputeThreadSizeY)),
            1
        },
        dsu.writes, dsu.nWrites,
        &ubo);
}

} // namespace epic::nls
