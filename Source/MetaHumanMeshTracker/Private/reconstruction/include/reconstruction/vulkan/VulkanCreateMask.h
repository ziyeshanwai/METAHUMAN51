// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/compute/common/VulkanComputeContext.h>
#include <nls/vulkan/compute/common/VulkanComputeBuffer.h>
#include <nls/vulkan/compute/common/VulkanComputeShaderPipeline.h>

namespace epic {
namespace nls {

/**
 * Calculate an image mask. border values are set to 0, saturated or almost black pixels are also masked out.
 * The mask can be used with undistortion/rectification to know which pixels are not useful for reconstruction.
 */
class VulkanCreateMask : public VulkanComputeContext::Kernel {

public:
    VulkanCreateMask(std::shared_ptr<VulkanComputeContext>& vulkanComputeContext)
        : VulkanComputeContext::Kernel(vulkanComputeContext, "ComputeCreateMask")
    {
        VulkanCreateMask::Init();
    }

    VulkanCreateMask(VulkanCreateMask&&) = default;

    bool Init() override;

    void CreateMask(
        std::shared_ptr<VulkanComputeBuffer> inputBuffer,
        std::shared_ptr<VulkanComputeBuffer> outputBuffer,
        VulkanComputeContext::HBatch batch = nullptr,
        float darkThreshold = 0.03f,
        float brightThreshold = 0.99f
    );
};

} //namespace nls
} //namespace epic
