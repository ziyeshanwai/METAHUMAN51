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
class VulkanStereoMatching : public VulkanComputeContext::Kernel {

public:
    VulkanStereoMatching(std::shared_ptr<VulkanComputeContext>& vulkanComputeContext)
        : VulkanComputeContext::Kernel(vulkanComputeContext, "ComputeStereoMatching")
    {
        VulkanStereoMatching::Init();
    }

    VulkanStereoMatching(VulkanStereoMatching&&) = default;

    bool Init() override;

    void Matching(
        std::shared_ptr<VulkanComputeBuffer> imgLeft,
        std::shared_ptr<VulkanComputeBuffer> imgRight,
        std::shared_ptr<VulkanComputeBuffer> disparityLeft,
        int numMatchingOffsets,
        VulkanComputeContext::HBatch batch = nullptr
    );
};

} //namespace nls
} //namespace epic
