// Copyright Epic Games, Inc. All Rights Reserved.

#include <reconstruction/vulkan/VulkanStereoUniquenessConstraint.h>
#include <nls/vulkan/common/VulkanShaderModule.h>
#include <nls/vulkan/common/VulkanDebugUtils.h>

#include <carbon/Common.h>

#include "shaders/embed_StereoUniquenessConstraint.comp.spv.h"
// include shader for push constant definition and size of thread
#include "shaders/StereoUniquenessConstraint.comp.h"
#include "shaders/StereoUniquenessConstraint.comp.refl.h"

#include <cmath>

namespace epic::nls {

bool VulkanStereoUniquenessConstraint::Init()
{
    return Kernel::Init< StereoUniquenessConstraintReflection, StereoUniquenessConstraintPushConstants >(SPV_StereoUniquenessConstraint_comp_spv, sizeof(SPV_StereoUniquenessConstraint_comp_spv));
    //    VulkanObjectName::Set(m_vulkanComputeContext->Device()->Device(), m_vulkanComputeShaderPipeline->Pipeline(), "StereoUniquenessConstraint:pipeline");
    //    VulkanObjectName::Set(m_vulkanComputeContext->Device()->Device(), m_vulkanComputeShaderPipeline->PipelineLayout(), "StereoUniquenessConstraint:pipelineLayout");
}

void VulkanStereoUniquenessConstraint::ApplyUniquenessConstraint(
    std::shared_ptr<VulkanComputeBuffer> inputDisparityBuffer,
    std::shared_ptr<VulkanComputeBuffer> otherInputDisparityBuffer,
    std::shared_ptr<VulkanComputeBuffer> outputDisparityBuffer,
    VulkanComputeContext::HBatch batch
)
{
    CARBON_PRECONDITION(inputDisparityBuffer, "input disparity buffer must be valid");
    CARBON_PRECONDITION(otherInputDisparityBuffer, "other input disparity buffer must be valid");
    CARBON_PRECONDITION(outputDisparityBuffer, "output disparity buffer must be valid");
    CARBON_PRECONDITION(inputDisparityBuffer->Width() == outputDisparityBuffer->Width(), "input and output disparity buffers must have same width");
    CARBON_PRECONDITION(inputDisparityBuffer->Height() == outputDisparityBuffer->Height(), "input and output disparity buffers must have same height");
    CARBON_PRECONDITION(inputDisparityBuffer->Width() == otherInputDisparityBuffer->Width(), "input and other input disparity buffers must have same width");
    CARBON_PRECONDITION(inputDisparityBuffer->Height() == otherInputDisparityBuffer->Height(), "input and other input disparity buffers must have same height");

    const int w = inputDisparityBuffer->Width();
    const int h = inputDisparityBuffer->Height();
    StereoUniquenessConstraintPushConstants ubo = {w, h };

    StereoUniquenessConstraintReflection::DescSet0_Update   dsu(inputDisparityBuffer->ManagedBuffer()->Buffer(), otherInputDisparityBuffer->ManagedBuffer()->Buffer(), outputDisparityBuffer->ManagedBuffer()->Buffer());

    Run(batch,
        {
            (uint32_t)std::ceil(w / float(StereoUniquenessConstraintComputeThreadSizeX)), 
            (uint32_t)std::ceil(h / float(StereoUniquenessConstraintComputeThreadSizeY)), 
            1
        },
        dsu.writes, dsu.nWrites,
        &ubo);
}

} // namespace epic::nls
