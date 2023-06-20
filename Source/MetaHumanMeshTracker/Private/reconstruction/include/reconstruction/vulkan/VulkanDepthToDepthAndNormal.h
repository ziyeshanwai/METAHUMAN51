// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/compute/common/VulkanComputeContext.h>
#include <nls/vulkan/compute/common/VulkanComputeBuffer.h>
#include <nls/vulkan/compute/common/VulkanComputeShaderPipeline.h>

namespace epic {
namespace nls {

/**
 * Convert depth to depth and normal
 */
class VulkanDepthToDepthAndNormal : public VulkanComputeContext::Kernel {

public:
    VulkanDepthToDepthAndNormal(std::shared_ptr<VulkanComputeContext>& vulkanComputeContext)
        : VulkanComputeContext::Kernel(vulkanComputeContext, "ComputeDepthToDepthAndNormal")
    {
        VulkanDepthToDepthAndNormal::Init();
    }

    VulkanDepthToDepthAndNormal(VulkanDepthToDepthAndNormal&&) = default;

    bool Init() override;

    void DepthToDepthAndNormal(
        std::shared_ptr<VulkanComputeBuffer> inputBuffer,
        std::shared_ptr<VulkanComputeBuffer> outputBuffer,
        float fx,
        float fy,
        float skew,
        float cx,
        float cy,
        VulkanComputeContext::HBatch batch = nullptr
    );
};

} //namespace nls
} //namespace epic
