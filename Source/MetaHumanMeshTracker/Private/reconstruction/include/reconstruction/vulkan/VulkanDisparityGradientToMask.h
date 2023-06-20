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
class VulkanDisparityGradientToMask : public VulkanComputeContext::Kernel {

public:
    VulkanDisparityGradientToMask(std::shared_ptr<VulkanComputeContext>& vulkanComputeContext)
        : VulkanComputeContext::Kernel(vulkanComputeContext, "ComputeDisparityGradientToMask")
    {
        VulkanDisparityGradientToMask::Init();
    }

    VulkanDisparityGradientToMask(VulkanDisparityGradientToMask&&) = default;

    bool Init() override;

    void DisparityGradientToMask(
        std::shared_ptr<VulkanComputeBuffer> inputBuffer,
        std::shared_ptr<VulkanComputeBuffer> outputBuffer,
        float threshold,
        VulkanComputeContext::HBatch batch = nullptr
    );
};

} //namespace nls
} //namespace epic
