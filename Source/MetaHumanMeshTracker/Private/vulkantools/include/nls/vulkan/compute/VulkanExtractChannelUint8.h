// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/compute/common/VulkanComputeContext.h>
#include <nls/vulkan/compute/common/VulkanComputeBuffer.h>
#include <nls/vulkan/compute/common/VulkanComputeShaderPipeline.h>

namespace epic {
namespace nls {

class VulkanExtractChannelUint8 : public VulkanComputeContext::Kernel {

public:
    VulkanExtractChannelUint8(std::shared_ptr<VulkanComputeContext>& vulkanComputeContext)
        : VulkanComputeContext::Kernel(vulkanComputeContext, "ComputeExtractChannelUint8")
    {
        VulkanExtractChannelUint8::Init();
    }

    VulkanExtractChannelUint8(VulkanExtractChannelUint8&&) = default;

    bool Init() override;

    /**
     * Extracts a channel from multi-channel byte image. Note that the inputBuffer needs to be padded to a multiple of 4 bytes.
     * @see RequiredBufferByteSize
     */
    void ExtractChannelUint8(
        std::shared_ptr<VulkanComputeBuffer> inputBuffer,
        std::shared_ptr<VulkanComputeBuffer> outputBuffer,
        int nInputChannels,
        int outputChannel,
        bool isSRGB,
        float scale = 1.0f / 255.f,
        VulkanComputeContext::HBatch batch = nullptr
    );

    /**
     * The shader to convert uint8 to float needs to first read a uint32 and then extract the float using bit shifts. Given
     * that full uint32 are read in the shader, the buffer needs to be padded accordingly to prevent a memory access error.
     * @returns the required buffer size for a byte image of size @p width, @p height, and @p numChannels
     */
    static int RequiredBufferByteSize(int width, int height, int numChannels) {
        const int numBytes = width * height * numChannels;
        return (numBytes % 4) ? (numBytes + 4 - (numBytes % 4)) : numBytes;
    }
};


} //namespace nls
} //namespace epic
