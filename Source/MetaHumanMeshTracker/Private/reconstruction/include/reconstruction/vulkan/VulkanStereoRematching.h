// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/compute/common/VulkanComputeContext.h>
#include <nls/vulkan/compute/common/VulkanComputeBuffer.h>
#include <nls/vulkan/compute/common/VulkanComputeShaderPipeline.h>

namespace epic {
namespace nls {

class VulkanStereoRematching : public VulkanComputeContext::Kernel {

public:
    VulkanStereoRematching(std::shared_ptr<VulkanComputeContext>& vulkanComputeContext)
        : VulkanComputeContext::Kernel(vulkanComputeContext, "ComputeStereoRematching")
    {
        VulkanStereoRematching::Init();
    }

    VulkanStereoRematching(VulkanStereoRematching&&) = default;

    bool Init() override;

    void Rematch(
        std::shared_ptr<VulkanComputeBuffer> imgLeft,
        std::shared_ptr<VulkanComputeBuffer> imgRight,
        std::shared_ptr<VulkanComputeBuffer> inputDisparityBuffer,
        std::shared_ptr<VulkanComputeBuffer> outputDisparityBuffer,
        VulkanComputeContext::HBatch batch = nullptr
    );
};

} //namespace nls
} //namespace epic
