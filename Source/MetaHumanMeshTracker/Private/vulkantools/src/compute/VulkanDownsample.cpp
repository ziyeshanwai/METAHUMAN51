// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/vulkan/compute/VulkanDownsample.h>
#include <nls/vulkan/common/VulkanShaderModule.h>
#include <nls/vulkan/common/VulkanDebugUtils.h>

#include <carbon/Common.h>

#include "shaders/embed_Downsample.comp.spv.h"
// include shader for push constant definition and size of thread
#include "shaders/Downsample.comp.h"
#include "shaders/Downsample.comp.refl.h"

#include <cmath>

namespace epic::nls {

bool VulkanDownsample::Init()
{
    return Kernel::Init< DownsampleReflection, DownsamplePushConstants >(SPV_Downsample_comp_spv, sizeof(SPV_Downsample_comp_spv));
}

void VulkanDownsample::Downsample(
    std::shared_ptr<VulkanComputeBuffer> inputBuffer,
    std::shared_ptr<VulkanComputeBuffer> outputBuffer,
    VulkanComputeContext::HBatch batch
)
{
    CARBON_PRECONDITION(inputBuffer->Width() == 2 * outputBuffer->Width(), "output buffer needs to have half the width of the input buffer");
    CARBON_PRECONDITION(inputBuffer->Height() == 2 * outputBuffer->Height(), "output buffer needs to have half the height of the input buffer");

    DownsamplePushConstants ubo = {outputBuffer->Width(), outputBuffer->Height() };

    DownsampleReflection::DescSet0_Update   dsu(inputBuffer->ManagedBuffer()->Buffer(), outputBuffer->ManagedBuffer()->Buffer());

    Run(batch,
        {
            (uint32_t)std::ceil(outputBuffer->Width() / float(DownsampleComputeThreadSizeX)), 
            (uint32_t)std::ceil(outputBuffer->Height() / float(DownsampleComputeThreadSizeY)),
            1
        },
        dsu.writes, dsu.nWrites,
        &ubo
    );
}

} // namespace epic::nls
