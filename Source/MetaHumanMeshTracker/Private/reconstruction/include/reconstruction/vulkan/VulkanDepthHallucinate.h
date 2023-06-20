// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/compute/common/VulkanComputeContext.h>
#include <nls/vulkan/compute/common/VulkanComputeBuffer.h>
#include <nls/vulkan/compute/common/VulkanComputeShaderPipeline.h>

namespace epic::nls {

/**
 * Convert disparity to depth
 */
class VulkanDepthHallucinate : public VulkanComputeContext::Kernel {

public:
    VulkanDepthHallucinate(std::shared_ptr<VulkanComputeContext>& vulkanComputeContext)
        : VulkanComputeContext::Kernel(vulkanComputeContext, "ComputeDepthHallucinate")
    {
        VulkanDepthHallucinate::Init();
    }

    VulkanDepthHallucinate(VulkanDepthHallucinate&&) = default;

    bool Init() override;

    void DepthHallucinate(
        std::shared_ptr<VulkanComputeBuffer> depthBuffer,
        std::shared_ptr<VulkanComputeBuffer> heightBuffer,
        std::shared_ptr<VulkanComputeBuffer> outputBuffer,
        float scale,
        VulkanComputeContext::HBatch batch = nullptr
    );
};

} //namespace epic::nls
