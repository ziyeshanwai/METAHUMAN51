// Copyright Epic Games, Inc. All Rights Reserved.

#include <reconstruction/vulkan/VulkanStereoRefine.h>
#include <nls/vulkan/common/VulkanShaderModule.h>
#include <nls/vulkan/common/VulkanDebugUtils.h>

#include <carbon/Common.h>

#include "shaders/embed_StereoRefine.comp.spv.h"
// include shader for push constant definition and size of thread
#include "shaders/StereoRefine.comp.h"
#include "shaders/StereoRefine.comp.refl.h"

#include <cmath>

namespace epic::nls {

bool VulkanStereoRefine::Init()
{
    return Kernel::Init< StereoRefineReflection, StereoRefinePushConstants >(SPV_StereoRefine_comp_spv, sizeof(SPV_StereoRefine_comp_spv));

//    VulkanObjectName::Set(m_vulkanComputeContext->Device()->Device(), m_vulkanComputeShaderPipeline->Pipeline(), "StereoRefine:pipeline");
//    VulkanObjectName::Set(m_vulkanComputeContext->Device()->Device(), m_vulkanComputeShaderPipeline->PipelineLayout(), "StereoRefine:pipelineLayout");
}

void VulkanStereoRefine::Refine(
    std::shared_ptr<VulkanComputeBuffer> imgLeft,
    std::shared_ptr<VulkanComputeBuffer> imgRight,
    std::shared_ptr<VulkanComputeBuffer> inputDisparity,
    std::shared_ptr<VulkanComputeBuffer> outputDisparity,
    float ws,
    VulkanComputeContext::HBatch batch
)
{
    CARBON_PRECONDITION(imgLeft, "imgLeft buffer must be valid");
    CARBON_PRECONDITION(imgRight, "imgRight buffer must be valid");
    CARBON_PRECONDITION(inputDisparity, "input disparity buffer must be valid");
    CARBON_PRECONDITION(outputDisparity, "output disparity buffer must be valid");

    const int w = imgLeft->Width();
    const int h = imgLeft->Height();

    CARBON_PRECONDITION(imgRight->Width() == w, "all buffers must have same width");
    CARBON_PRECONDITION(imgRight->Height() == h, "all buffers must have same height");
    CARBON_PRECONDITION(inputDisparity->Width() == w, "all buffers must have same width");
    CARBON_PRECONDITION(inputDisparity->Height() == h, "all buffers must have same height");
    CARBON_PRECONDITION(outputDisparity->Width() == w, "all buffers must have same width");
    CARBON_PRECONDITION(outputDisparity->Height() == h, "all buffers must have same height");

    StereoRefinePushConstants ubo = {w, h, ws };

    StereoRefineReflection::DescSet0_Update dsu(imgLeft->ManagedBuffer()->Buffer(), imgRight->ManagedBuffer()->Buffer(),
                                                inputDisparity->ManagedBuffer()->Buffer(), outputDisparity->ManagedBuffer()->Buffer());

    Run(batch,
        {
            (uint32_t)std::ceil(w / float(StereoRefineComputeThreadSizeX)), 
            (uint32_t)std::ceil(h / float(StereoRefineComputeThreadSizeY)), 
            1
        },
        dsu.writes, dsu.nWrites,
        & ubo);
}

} // namespace epic::nls
