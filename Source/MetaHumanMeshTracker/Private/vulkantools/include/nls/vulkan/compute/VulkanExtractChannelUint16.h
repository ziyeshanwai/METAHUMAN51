// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/compute/common/VulkanComputeContext.h>
#include <nls/vulkan/compute/common/VulkanComputeBuffer.h>
#include <nls/vulkan/compute/common/VulkanComputeShaderPipeline.h>

namespace epic {
namespace nls {

class VulkanExtractChannelUint16 : public VulkanComputeContext::Kernel {

public:
    VulkanExtractChannelUint16(std::shared_ptr<VulkanComputeContext>& vulkanComputeContext)
        : VulkanComputeContext::Kernel(vulkanComputeContext, "ComputeExtractChannelUint16")
    {
        VulkanExtractChannelUint16::Init();
    }

    VulkanExtractChannelUint16(VulkanExtractChannelUint16&&) = default;

    bool Init() override;

    /**
     * Extracts a channel from multi-channel uint16 image. Note that the inputBuffer needs to be padded to a multiple of 4 bytes.
     * @see RequiredBufferByteSize
     */
    void ExtractChannelUint16(
        std::shared_ptr<VulkanComputeBuffer> inputBuffer,
        std::shared_ptr<VulkanComputeBuffer> outputBuffer,
        int nInputChannels,
        int outputChannel,
        bool isSRGB,
        float scale = 1.0f / 65535.0f,
        VulkanComputeContext::HBatch batch = nullptr
    );

    /**
     * The shader to convert uint16 to float needs to first read a uint32 and then extract the float using bit shifts. Given
     * that full uint32 are read in the shader, the buffer needs to be padded accordingly to prevent a memory access error.
     * @returns the required buffer size for a byte image of size @p width, @p height, and @p numChannels
     */
    static int RequiredBufferByteSize(int width, int height, int numChannels) {
        const int numBytes = 2 * (width * height * numChannels);
        return (numBytes % 4) ? (numBytes + 4 - (numBytes % 4)) : numBytes;
    }
};


} //namespace nls
} //namespace epic
