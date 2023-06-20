// Copyright Epic Games, Inc. All Rights Reserved.

#include <reconstruction/vulkan/VulkanCreateMask.h>
#include <nls/vulkan/common/VulkanShaderModule.h>

#include <carbon/Common.h>

#include "shaders/embed_CreateMask.comp.spv.h"
// include shader for push constant definition and size of thread
#include "shaders/CreateMask.comp.h"
#include "shaders/CreateMask.comp.refl.h"

#include <cmath>

namespace epic::nls {

bool VulkanCreateMask::Init()
{
    return Kernel::Init< CreateMaskReflection, CreateMaskPushConstants >(SPV_CreateMask_comp_spv, sizeof(SPV_CreateMask_comp_spv));
}

void VulkanCreateMask::CreateMask(
    std::shared_ptr<VulkanComputeBuffer> inputBuffer,
    std::shared_ptr<VulkanComputeBuffer> outputBuffer,
    VulkanComputeContext::HBatch batch,
    float darkThreshold,
    float brightThreshold
)
{
    CARBON_PRECONDITION(inputBuffer, "input buffer must be valid");
    CARBON_PRECONDITION(outputBuffer, "output buffer must be valid");
    CARBON_PRECONDITION(inputBuffer->Width() == outputBuffer->Width(), "input and output buffer must have same width");
    CARBON_PRECONDITION(inputBuffer->Height() == outputBuffer->Height(), "input and output buffer must have same height");

    const int w = inputBuffer->Width();
    const int h = inputBuffer->Height();
    CreateMaskPushConstants ubo = {w, h, darkThreshold, brightThreshold };

    CreateMaskReflection::DescSet0_Update dsu(inputBuffer->ManagedBuffer()->Buffer(), outputBuffer->ManagedBuffer()->Buffer());

    Run(batch,
        {
            (uint32_t)std::ceil(w / float(CreateMaskComputeThreadSizeX)),
            (uint32_t)std::ceil(h / float(CreateMaskComputeThreadSizeY)),
            1
        },
        dsu.writes, dsu.nWrites,
        &ubo);
}

} // namespace epic::nls
