// Copyright Epic Games, Inc. All Rights Reserved.

#include <reconstruction/vulkan/VulkanStereoMaskConstraint.h>
#include <nls/vulkan/common/VulkanShaderModule.h>
#include <nls/vulkan/common/VulkanDebugUtils.h>

#include <carbon/Common.h>

#include "shaders/embed_StereoMaskConstraint.comp.spv.h"
// include shader for push constant definition and size of thread
#include "shaders/StereoMaskConstraint.comp.h"
#include "shaders/StereoMaskConstraint.comp.refl.h"

#include <cmath>

namespace epic::nls {

bool VulkanStereoMaskConstraint::Init()
{
    return Kernel::Init< StereoMaskConstraintReflection, StereoMaskConstraintPushConstants >(SPV_StereoMaskConstraint_comp_spv, sizeof(SPV_StereoMaskConstraint_comp_spv));

//        VulkanObjectName::Set(m_vulkanComputeContext->Device()->Device(), m_vulkanComputeShaderPipeline->Pipeline(), "StereoMaskConstraint:pipeline");
//        VulkanObjectName::Set(m_vulkanComputeContext->Device()->Device(), m_vulkanComputeShaderPipeline->PipelineLayout(), "StereoMaskConstraint:pipelineLayout");
}

void VulkanStereoMaskConstraint::MaskConstraint(
    std::shared_ptr<VulkanComputeBuffer> disparityBuffer,
    std::shared_ptr<VulkanComputeBuffer> leftBuffer,
    std::shared_ptr<VulkanComputeBuffer> rightBuffer,
    VulkanComputeContext::HBatch batch
)
{
    CARBON_PRECONDITION(disparityBuffer, "disparity buffer must be valid");
    CARBON_PRECONDITION(leftBuffer, "left buffer must be valid");
    CARBON_PRECONDITION(rightBuffer, "right buffer must be valid");
    CARBON_PRECONDITION(disparityBuffer->Width() == leftBuffer->Width(), "disparity and right buffer must have same width");
    CARBON_PRECONDITION(disparityBuffer->Height() == leftBuffer->Height(), "disparity and left buffer must have same height");
    CARBON_PRECONDITION(disparityBuffer->Height() == rightBuffer->Height(), "disparity and right buffer must have same height");
    
    const int w = disparityBuffer->Width();
    const int h = disparityBuffer->Height();
    StereoMaskConstraintPushConstants ubo = {w, h, rightBuffer->Width() };

    StereoMaskConstraintReflection::DescSet0_Update dsu(disparityBuffer->ManagedBuffer()->Buffer(), leftBuffer->ManagedBuffer()->Buffer(), rightBuffer->ManagedBuffer()->Buffer());

    Run(batch,
        {
            (uint32_t)std::ceil(w / float(StereoMaskConstraintComputeThreadSizeX)), 
            (uint32_t)std::ceil(h / float(StereoMaskConstraintComputeThreadSizeY)), 
            1
        },
        dsu.writes, dsu.nWrites,
        &ubo);
}

} // namespace epic::nls
