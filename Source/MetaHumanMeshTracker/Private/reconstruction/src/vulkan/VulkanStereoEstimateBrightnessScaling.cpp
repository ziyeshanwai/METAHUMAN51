// Copyright Epic Games, Inc. All Rights Reserved.

#include <reconstruction/vulkan/VulkanStereoEstimateBrightnessScaling.h>
#include <nls/vulkan/common/VulkanShaderModule.h>
#include <nls/vulkan/common/VulkanDebugUtils.h>

#include <carbon/Common.h>

#include "shaders/embed_StereoEstimateBrightnessScaling.comp.spv.h"
// include shader for push constant definition and size of thread
#include "shaders/StereoEstimateBrightnessScaling.comp.h"
#include "shaders/StereoEstimateBrightnessScaling.comp.refl.h"

#include <cmath>

namespace epic::nls {

bool VulkanStereoEstimateBrightnessScaling::Init()
{
    return Kernel::Init< StereoEstimateBrightnessScalingReflection, StereoEstimateBrightnessScalingPushConstants >(SPV_StereoEstimateBrightnessScaling_comp_spv, sizeof(SPV_StereoEstimateBrightnessScaling_comp_spv));

//    VulkanObjectName::Set(m_vulkanComputeContext->Device()->Device(), m_vulkanComputeShaderPipeline->Pipeline(), "StereoEstimateBrightnessScaling:pipeline");
//    VulkanObjectName::Set(m_vulkanComputeContext->Device()->Device(), m_vulkanComputeShaderPipeline->PipelineLayout(), "StereoEstimateBrightnessScaling:pipelineLayout");
}

void VulkanStereoEstimateBrightnessScaling::EstimateBrightnessScaling(
    std::shared_ptr<VulkanComputeBuffer> disparityBuffer,
    std::shared_ptr<VulkanComputeBuffer> srcBuffer,
    std::shared_ptr<VulkanComputeBuffer> targetBuffer,
    std::shared_ptr<VulkanComputeBuffer> scalingImageBuffer,
    VulkanComputeContext::HBatch batch
)
{
    CARBON_PRECONDITION(disparityBuffer, "disparity buffer must be valid");
    CARBON_PRECONDITION(srcBuffer, "src buffer must be valid");
    CARBON_PRECONDITION(targetBuffer, "target buffer must be valid");
    CARBON_PRECONDITION(scalingImageBuffer, "scaling buffer must be valid");

    CARBON_PRECONDITION(disparityBuffer->Width() == srcBuffer->Width(), "disparity and src buffer must have same width");
    CARBON_PRECONDITION(disparityBuffer->Height() == srcBuffer->Height(), "disparity and src buffer must have same height");
    CARBON_PRECONDITION(disparityBuffer->Width() == targetBuffer->Width(), "disparity and target buffer must have same width");
    CARBON_PRECONDITION(disparityBuffer->Height() == targetBuffer->Height(), "disparity and target buffer must have same height");
    CARBON_PRECONDITION(disparityBuffer->Width() == scalingImageBuffer->Width(), "disparity and scaling buffer must have same width");
    CARBON_PRECONDITION(disparityBuffer->Height() == scalingImageBuffer->Height(), "disparity and scaling buffer must have same height");

    const int w = disparityBuffer->Width();
    const int h = disparityBuffer->Height();
    StereoEstimateBrightnessScalingPushConstants ubo = {w, h };

    StereoEstimateBrightnessScalingReflection::DescSet0_Update  dsu(
                                                                    srcBuffer->ManagedBuffer()->Buffer(),
                                                                    targetBuffer->ManagedBuffer()->Buffer(),
                                                                    disparityBuffer->ManagedBuffer()->Buffer(),
                                                                    scalingImageBuffer->ManagedBuffer()->Buffer());

    Run(batch,
        {
            (uint32_t)std::ceil(w / float(StereoEstimateBrightnessScalingComputeThreadSizeX)), 
            (uint32_t)std::ceil(h / float(StereoEstimateBrightnessScalingComputeThreadSizeY)), 
            1
        },
        dsu.writes, dsu.nWrites,
        &ubo);
}

} // namespace epic::nls
