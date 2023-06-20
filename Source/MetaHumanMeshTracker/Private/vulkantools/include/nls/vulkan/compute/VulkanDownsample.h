// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/compute/common/VulkanComputeContext.h>
#include <nls/vulkan/compute/common/VulkanComputeBuffer.h>
#include <nls/vulkan/compute/common/VulkanComputeShaderPipeline.h>

namespace epic {
namespace nls {

class VulkanDownsample : public VulkanComputeContext::Kernel {

public:
    VulkanDownsample(std::shared_ptr<VulkanComputeContext>& vulkanComputeContext)
        : VulkanComputeContext::Kernel(vulkanComputeContext, "ComputeDownsample")
    {
        VulkanDownsample::Init();
    }

    VulkanDownsample(VulkanDownsample&&) = default;

    bool Init() override;

    void Downsample(
        std::shared_ptr<VulkanComputeBuffer> inputBuffer,
        std::shared_ptr<VulkanComputeBuffer> outputBuffer,
        VulkanComputeContext::HBatch batch = nullptr
    );
};


} //namespace nls
} //namespace epic
