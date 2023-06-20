// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/compute/common/VulkanComputeContext.h>
#include <nls/vulkan/compute/common/VulkanComputeBuffer.h>
#include <nls/vulkan/compute/common/VulkanComputeShaderPipeline.h>

namespace epic {
namespace nls {

/**
 * Convert disparity to depth
 */
class VulkanDisparityToDepth : public VulkanComputeContext::Kernel {

public:
    VulkanDisparityToDepth(std::shared_ptr<VulkanComputeContext>& vulkanComputeContext)
        : VulkanComputeContext::Kernel(vulkanComputeContext, "ComputeDisparityToDepth")
    {
        VulkanDisparityToDepth::Init();
    }

    VulkanDisparityToDepth(VulkanDisparityToDepth&&) = default;

    bool Init() override;

    void DisparityToDepth(
        std::shared_ptr<VulkanComputeBuffer> inputBuffer,
        std::shared_ptr<VulkanComputeBuffer> outputBuffer,
        float f,
        float b,
        float offset,
        float scaleDisparity,
        VulkanComputeContext::HBatch batch = nullptr
    );
};

} //namespace nls
} //namespace epic
