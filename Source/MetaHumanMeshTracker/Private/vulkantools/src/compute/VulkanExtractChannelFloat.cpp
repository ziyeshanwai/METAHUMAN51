// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/vulkan/compute/VulkanExtractChannelFloat.h>
#include <nls/vulkan/common/VulkanShaderModule.h>

#include <carbon/Common.h>

#include "shaders/embed_ExtractChannelFloat.comp.spv.h"
// include shader for push constant definition and size of thread
#include "shaders/ExtractChannelFloat.comp.h"
#include "shaders/ExtractChannelFloat.comp.refl.h"

#include <cmath>

namespace epic::nls {

bool VulkanExtractChannelFloat::Init()
{
    return Kernel::Init< ExtractChannelFloatReflection, ExtractChannelFloatPushConstants >(SPV_ExtractChannelFloat_comp_spv, sizeof(SPV_ExtractChannelFloat_comp_spv));
}

void VulkanExtractChannelFloat::ExtractChannelFloat(
    std::shared_ptr<VulkanComputeBuffer> inputBuffer,
    std::shared_ptr<VulkanComputeBuffer> outputBuffer,
    int nInputChannels,
    int outputChannel,
    bool isSRGB,
    float scale,
    VulkanComputeContext::HBatch batch
)
{
    CARBON_PRECONDITION(inputBuffer, "input buffer must be valid");
    CARBON_PRECONDITION(outputBuffer, "output buffer must be valid");
    CARBON_PRECONDITION(inputBuffer->Width() == nInputChannels * outputBuffer->Width(), "input buffer must have a multiple ({}) width of the input buffer", nInputChannels);
    CARBON_PRECONDITION(inputBuffer->Height() == outputBuffer->Height(), "input and output buffer must have same height");

    const int w = inputBuffer->Width();
    const int h = inputBuffer->Height();

    ExtractChannelFloatPushConstants ubo = {w, h, nInputChannels, outputChannel, int(isSRGB), scale };

    ExtractChannelFloatReflection::DescSet0_Update   dsu(inputBuffer->ManagedBuffer()->Buffer(), outputBuffer->ManagedBuffer()->Buffer());

    Run(batch,
        {
            (uint32_t)std::ceil(w / float(ExtractChannelFloatComputeThreadSizeX)),
            (uint32_t)std::ceil(h / float(ExtractChannelFloatComputeThreadSizeY)),
            1
        },
        dsu.writes, dsu.nWrites,
        &ubo
    );
}

} // namespace epic::nls
