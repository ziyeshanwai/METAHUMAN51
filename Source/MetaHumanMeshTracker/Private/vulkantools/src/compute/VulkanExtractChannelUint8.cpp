// Copyright Epic Games, Inc. All Rights Reserved.

#include <nls/vulkan/compute/VulkanExtractChannelUint8.h>
#include <nls/vulkan/common/VulkanShaderModule.h>

#include <carbon/Common.h>

#include "shaders/embed_ExtractChannelUint8.comp.spv.h"
// include shader for push constant definition and size of thread
#include "shaders/ExtractChannelUint8.comp.h"
#include "shaders/ExtractChannelUint8.comp.refl.h"

#include <cmath>

namespace epic::nls {

bool VulkanExtractChannelUint8::Init()
{
    return Kernel::Init< ExtractChannelUint8Reflection, ExtractChannelUint8PushConstants >(SPV_ExtractChannelUint8_comp_spv, sizeof(SPV_ExtractChannelUint8_comp_spv));
}

void VulkanExtractChannelUint8::ExtractChannelUint8(
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
    const int expectedBytes = RequiredBufferByteSize(outputBuffer->Width(), outputBuffer->Height(), nInputChannels);
    CARBON_PRECONDITION(inputBuffer->SizeInBytes()  == expectedBytes, "expected {} padded bytes, but got {}", expectedBytes, inputBuffer->SizeInBytes());

    const int w = outputBuffer->Width();
    const int h = outputBuffer->Height();

    ExtractChannelUint8PushConstants ubo = {w, h, nInputChannels, outputChannel, int(isSRGB), scale };

    ExtractChannelUint8Reflection::DescSet0_Update   dsu(inputBuffer->ManagedBuffer()->Buffer(), outputBuffer->ManagedBuffer()->Buffer());

    Run(batch,
        {
            (uint32_t)std::ceil(w / float(ExtractChannelUint8ComputeThreadSizeX)),
            (uint32_t)std::ceil(h / float(ExtractChannelUint8ComputeThreadSizeY)),
            1
        },
        dsu.writes, dsu.nWrites,
        &ubo
    );
}

} // namespace epic::nls
