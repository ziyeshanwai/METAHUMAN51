// Copyright Epic Games, Inc. All Rights Reserved.

#include <reconstruction/vulkan/VulkanStereoUpsample.h>
#include <nls/vulkan/common/VulkanShaderModule.h>
#include <nls/vulkan/common/VulkanDebugUtils.h>

#include <carbon/Common.h>

#include "shaders/embed_StereoUpsample.comp.spv.h"
// include shader for push constant definition and size of thread
#include "shaders/StereoUpsample.comp.h"
#include "shaders/StereoUpsample.comp.refl.h"

#include <cmath>

namespace epic::nls {

bool VulkanStereoUpsample::Init()
{
    return Kernel::Init< StereoUpsampleReflection, StereoUpsamplePushConstants >(SPV_StereoUpsample_comp_spv, sizeof(SPV_StereoUpsample_comp_spv));

//        VulkanObjectName::Set(m_vulkanComputeContext->Device()->Device(), m_vulkanComputeShaderPipeline->Pipeline(), "StereoUpsample:pipeline");
//        VulkanObjectName::Set(m_vulkanComputeContext->Device()->Device(), m_vulkanComputeShaderPipeline->PipelineLayout(), "StereoUpsample:pipelineLayout");
}

void VulkanStereoUpsample::Upsample(
    std::shared_ptr<VulkanComputeBuffer> inputBuffer,
    std::shared_ptr<VulkanComputeBuffer> outputBuffer,
    VulkanComputeContext::HBatch batch
)
{
    CARBON_PRECONDITION(inputBuffer, "input buffer must be valid");
    CARBON_PRECONDITION(outputBuffer, "output buffer must be valid");
    CARBON_PRECONDITION(2 * inputBuffer->Width() == outputBuffer->Width(), "input buffer must have half the width of the output buffer");
    CARBON_PRECONDITION(2 * inputBuffer->Height() == outputBuffer->Height(), "input buffer must have half the height of the output buffer");

    const int w = outputBuffer->Width();
    const int h = outputBuffer->Height();
    StereoUpsamplePushConstants ubo = {w, h };

    StereoUpsampleReflection::DescSet0_Update   dsu(inputBuffer->ManagedBuffer()->Buffer(), outputBuffer->ManagedBuffer()->Buffer());

    Run(batch,
        {
            (uint32_t)std::ceil(w / float(StereoUpsampleComputeThreadSizeX)), 
            (uint32_t)std::ceil(h / float(StereoUpsampleComputeThreadSizeY)), 
            1
        },
        dsu.writes, dsu.nWrites,
        &ubo);
}

} // namespace epic::nls
