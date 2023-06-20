// Copyright Epic Games, Inc. All Rights Reserved.

#include <reconstruction/vulkan/VulkanStereoRematching.h>
#include <nls/vulkan/common/VulkanShaderModule.h>
#include <nls/vulkan/common/VulkanDebugUtils.h>

#include <carbon/Common.h>

#include "shaders/embed_StereoRematching.comp.spv.h"
// include shader for push constant definition and size of thread
#include "shaders/StereoRematching.comp.h"
#include "shaders/StereoRematching.comp.refl.h"

#include <cmath>

namespace epic::nls {

bool VulkanStereoRematching::Init()
{
    return Kernel::Init< StereoRematchingReflection, StereoRematchingPushConstants >(SPV_StereoRematching_comp_spv, sizeof(SPV_StereoRematching_comp_spv));

//    VulkanObjectName::Set(m_vulkanComputeContext->Device()->Device(), m_vulkanComputeShaderPipeline->Pipeline(), "StereoRematching:pipeline");
//    VulkanObjectName::Set(m_vulkanComputeContext->Device()->Device(), m_vulkanComputeShaderPipeline->PipelineLayout(), "StereoRematching:pipelineLayout");
}

void VulkanStereoRematching::Rematch(
    std::shared_ptr<VulkanComputeBuffer> imgLeft,
    std::shared_ptr<VulkanComputeBuffer> imgRight,
    std::shared_ptr<VulkanComputeBuffer> inputDisparity,
    std::shared_ptr<VulkanComputeBuffer> outputDisparity,
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

    StereoRematchingPushConstants ubo = {w, h };

    StereoRematchingReflection::DescSet0_Update dsu(imgLeft->ManagedBuffer()->Buffer(), imgRight->ManagedBuffer()->Buffer(), inputDisparity->ManagedBuffer()->Buffer(), outputDisparity->ManagedBuffer()->Buffer());

    Run(batch,
        {
            (uint32_t)std::ceil(w / float(StereoRematchingComputeThreadSizeX)),
            (uint32_t)std::ceil(h / float(StereoRematchingComputeThreadSizeY)),
            1
        },
        dsu.writes, dsu.nWrites,
        &ubo);
}

} // namespace epic::nls
