// Copyright Epic Games, Inc. All Rights Reserved.

#include <reconstruction/vulkan/VulkanStereoMatching.h>
#include <nls/vulkan/common/VulkanShaderModule.h>
#include <nls/vulkan/common/VulkanDebugUtils.h>

#include <carbon/Common.h>

#include "shaders/embed_StereoMatching.comp.spv.h"
// include shader for push constant definition and size of thread
#include "shaders/StereoMatching.comp.h"
#include "shaders/StereoMatching.comp.refl.h"

#include <cmath>

namespace epic::nls {

bool VulkanStereoMatching::Init()
{
    return Kernel::Init< StereoMatchingReflection, StereoMatchingPushConstants >(SPV_StereoMatching_comp_spv, sizeof(SPV_StereoMatching_comp_spv));

    //VulkanObjectName::Set(m_vulkanComputeContext->Device()->Device(), m_vulkanComputeShaderPipeline->Pipeline(), "StereoMatching:pipeline");
    //VulkanObjectName::Set(m_vulkanComputeContext->Device()->Device(), m_vulkanComputeShaderPipeline->PipelineLayout(), "StereoMatching:pipelineLayout");
}

void VulkanStereoMatching::Matching(
    std::shared_ptr<VulkanComputeBuffer> imgLeft,
    std::shared_ptr<VulkanComputeBuffer> imgRight,
    std::shared_ptr<VulkanComputeBuffer> disparityLeft,
    int numMatchingOffsets,
    VulkanComputeContext::HBatch batch
)
{
    CARBON_PRECONDITION(imgLeft, "imgLeft buffer must be valid");
    CARBON_PRECONDITION(imgRight, "imgRight buffer must be valid");
    CARBON_PRECONDITION(disparityLeft, "disparityLeft buffer must be valid");

    const int w = imgLeft->Width();
    const int h = imgLeft->Height();

    CARBON_PRECONDITION(imgRight->Width() == w, "all buffers must have same width");
    CARBON_PRECONDITION(imgRight->Height() == h, "all buffers must have same height");
    CARBON_PRECONDITION(disparityLeft->Width() == w, "all buffers must have same width");
    CARBON_PRECONDITION(disparityLeft->Height() == h, "all buffers must have same height");

    StereoMatchingPushConstants ubo = {w, h, numMatchingOffsets };

    StereoMatchingReflection::DescSet0_Update dsu(imgLeft->ManagedBuffer()->Buffer(), imgRight->ManagedBuffer()->Buffer(), disparityLeft->ManagedBuffer()->Buffer());

    Run(batch,
        {
            (uint32_t)std::ceil(w / float(StereoMatchingComputeThreadSizeX)), 
            (uint32_t)std::ceil(h / float(StereoMatchingComputeThreadSizeY)), 
            1
        },
        dsu.writes, dsu.nWrites,
        &ubo);
}

} // namespace epic::nls
