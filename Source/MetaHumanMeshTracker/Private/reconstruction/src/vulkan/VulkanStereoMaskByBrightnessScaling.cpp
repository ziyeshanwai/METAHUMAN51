// Copyright Epic Games, Inc. All Rights Reserved.

#include <reconstruction/vulkan/VulkanStereoMaskByBrightnessScaling.h>
#include <nls/vulkan/common/VulkanShaderModule.h>
#include <nls/vulkan/common/VulkanDebugUtils.h>

#include <carbon/Common.h>

#include "shaders/embed_StereoMaskByBrightnessScaling.comp.spv.h"
// include shader for push constant definition and size of thread
#include "shaders/StereoMaskByBrightnessScaling.comp.h"
#include "shaders/StereoMaskByBrightnessScaling.comp.refl.h"

#include <cmath>

namespace epic::nls {

bool VulkanStereoMaskByBrightnessScaling::Init()
{
    return Kernel::Init< StereoMaskByBrightnessScalingReflection, StereoMaskByBrightnessScalingPushConstants >(SPV_StereoMaskByBrightnessScaling_comp_spv, sizeof(SPV_StereoMaskByBrightnessScaling_comp_spv));

//    VulkanObjectName::Set(m_vulkanComputeContext->Device()->Device(), m_vulkanComputeShaderPipeline->Pipeline(), "StereoMaskByBrightnessScaling:pipeline");
//    VulkanObjectName::Set(m_vulkanComputeContext->Device()->Device(), m_vulkanComputeShaderPipeline->PipelineLayout(), "StereoMaskByBrightnessScaling:pipelineLayout");
}

void VulkanStereoMaskByBrightnessScaling::MaskByBrightnessScaling(
    std::shared_ptr<VulkanComputeBuffer> disparityBuffer,
    std::shared_ptr<VulkanComputeBuffer> srcBuffer,
    std::shared_ptr<VulkanComputeBuffer> targetBuffer,
    float mainScale,
    float scaleDifferenceThreshold,
    VulkanComputeContext::HBatch batch
)
{
    CARBON_PRECONDITION(disparityBuffer, "disparity buffer must be valid");
    CARBON_PRECONDITION(srcBuffer, "src buffer must be valid");
    CARBON_PRECONDITION(targetBuffer, "target buffer must be valid");

    CARBON_PRECONDITION(disparityBuffer->Width() == srcBuffer->Width(), "disparity and src buffer must have same width");
    CARBON_PRECONDITION(disparityBuffer->Height() == srcBuffer->Height(), "disparity and src buffer must have same height");
    CARBON_PRECONDITION(srcBuffer->Width() == targetBuffer->Width(), "src and target buffer must have same width");
    CARBON_PRECONDITION(srcBuffer->Height() == targetBuffer->Height(), "src and target buffer must have same height");

    const int w = disparityBuffer->Width();
    const int h = disparityBuffer->Height();
    StereoMaskByBrightnessScalingPushConstants ubo = {w, h, mainScale, scaleDifferenceThreshold };

    StereoMaskByBrightnessScalingReflection::DescSet0_Update    dsu(srcBuffer->ManagedBuffer()->Buffer(),
                                                                    targetBuffer->ManagedBuffer()->Buffer(),
                                                                    disparityBuffer->ManagedBuffer()->Buffer());

    Run(batch,
        {
            (uint32_t)std::ceil(w / float(StereoMaskByBrightnessScalingComputeThreadSizeX)), 
            (uint32_t)std::ceil(h / float(StereoMaskByBrightnessScalingComputeThreadSizeY)), 
            1
        },
        dsu.writes, dsu.nWrites,
        &ubo);
}

} // namespace epic::nls
