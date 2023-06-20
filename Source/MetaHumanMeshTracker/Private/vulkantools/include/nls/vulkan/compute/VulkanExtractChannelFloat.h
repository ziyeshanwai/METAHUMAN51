// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/compute/common/VulkanComputeContext.h>
#include <nls/vulkan/compute/common/VulkanComputeBuffer.h>
#include <nls/vulkan/compute/common/VulkanComputeShaderPipeline.h>

namespace epic {
namespace nls {

class VulkanExtractChannelFloat : public VulkanComputeContext::Kernel {

public:
    VulkanExtractChannelFloat(std::shared_ptr<VulkanComputeContext>& vulkanComputeContext)
        : VulkanComputeContext::Kernel(vulkanComputeContext, "ComputeExtractChannelFloat")
    {
        VulkanExtractChannelFloat::Init();
    }

    VulkanExtractChannelFloat(VulkanExtractChannelFloat&&) = default;

    bool Init() override;

    // extracts a channel from multi-channel float image
    void ExtractChannelFloat(
        std::shared_ptr<VulkanComputeBuffer> inputBuffer,
        std::shared_ptr<VulkanComputeBuffer> outputBuffer,
        int nInputChannels,
        int outputChannel,
        bool isSRGB,
        float scale = float(1.0),
        VulkanComputeContext::HBatch batch = nullptr
    );
};


} //namespace nls
} //namespace epic
