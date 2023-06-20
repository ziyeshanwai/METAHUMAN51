// Copyright Epic Games, Inc. All Rights Reserved.

#include <reconstruction/vulkan/VulkanDisparityGradientToMask.h>
#include <nls/vulkan/common/VulkanShaderModule.h>

#include <carbon/Common.h>

#include "shaders/embed_DisparityGradientToMask.comp.spv.h"
// include shader for push constant definition and size of thread
#include "shaders/DisparityGradientToMask.comp.h"
#include "shaders/DisparityGradientToMask.comp.refl.h"

#include <cmath>

namespace epic::nls {


bool VulkanDisparityGradientToMask::Init()
{
    return Kernel::Init< DisparityGradientToMaskReflection, DisparityGradientToMaskPushConstants >(SPV_DisparityGradientToMask_comp_spv, sizeof(SPV_DisparityGradientToMask_comp_spv));
}

void VulkanDisparityGradientToMask::DisparityGradientToMask(
    std::shared_ptr<VulkanComputeBuffer> inputBuffer,
    std::shared_ptr<VulkanComputeBuffer> outputBuffer,
    float threshold,
    VulkanComputeContext::HBatch batch
)
{
    CARBON_PRECONDITION(inputBuffer, "input buffer must be valid");
    CARBON_PRECONDITION(outputBuffer, "output buffer must be valid");
    CARBON_PRECONDITION(inputBuffer->Width() == outputBuffer->Width(), "input and output buffer must have same width");
    CARBON_PRECONDITION(inputBuffer->Height() == outputBuffer->Height(), "input and output buffer must have same height");

    const int w = inputBuffer->Width();
    const int h = inputBuffer->Height();
    DisparityGradientToMaskPushConstants ubo = {w, h, threshold };

    DisparityGradientToMaskReflection::DescSet0_Update dsu(inputBuffer->ManagedBuffer()->Buffer(), outputBuffer->ManagedBuffer()->Buffer());

    Run(batch,
        {
            (uint32_t)std::ceil(w / float(DisparityGradientToMaskComputeThreadSizeX)),
            (uint32_t)std::ceil(h / float(DisparityGradientToMaskComputeThreadSizeY)),
            1
        },
        dsu.writes, dsu.nWrites,
        &ubo);
}

} // namespace epic::nls
