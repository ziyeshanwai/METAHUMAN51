// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/compute/common/VulkanComputeContext.h>
#include <nls/vulkan/compute/common/VulkanComputeBuffer.h>
#include <nls/vulkan/compute/common/VulkanComputeShaderPipeline.h>

namespace epic {
namespace nls {

/**
 * Run initial stereo matching between two images.
 */
class VulkanStereoInitialMatching : public VulkanComputeContext::Kernel {

public:
    VulkanStereoInitialMatching(std::shared_ptr<VulkanComputeContext>& vulkanComputeContext)
        : VulkanComputeContext::Kernel(vulkanComputeContext, "ComputeStereoInitialMatching")
    {
        VulkanStereoInitialMatching::Init();
    }
    
    VulkanStereoInitialMatching(VulkanStereoInitialMatching&&) = default;

    bool Init() override;

    void InitialMatching(
        std::shared_ptr<VulkanComputeBuffer> imgLeft,
        std::shared_ptr<VulkanComputeBuffer> imgRight,
        std::shared_ptr<VulkanComputeBuffer> disparityLeft,
        int minimumDisparity,
        int maximumDisparity,
        VulkanComputeContext::HBatch batch = nullptr
    );
};

} //namespace nls
} //namespace epic
