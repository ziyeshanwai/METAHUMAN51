// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/vulkan/compute/common/VulkanComputeContext.h>
#include <nls/vulkan/compute/common/VulkanComputeBuffer.h>
#include <nls/vulkan/compute/common/VulkanComputeShaderPipeline.h>

namespace epic {
namespace nls {

class VulkanStereoOrderingConstraint : public VulkanComputeContext::Kernel {

public:
    VulkanStereoOrderingConstraint(std::shared_ptr<VulkanComputeContext>& vulkanComputeContext)
        : VulkanComputeContext::Kernel(vulkanComputeContext, "ComputeStereoOrderingConstraint")
    {
        VulkanStereoOrderingConstraint::Init();
    }

    VulkanStereoOrderingConstraint(VulkanStereoOrderingConstraint&&) = default;

    bool Init() override;

    void ApplyOrderingConstraint(
        std::shared_ptr<VulkanComputeBuffer> inputDisparityBuffer,
        std::shared_ptr<VulkanComputeBuffer> outputDisparityBuffer,
        VulkanComputeContext::HBatch batch = nullptr
    );
};

} //namespace nls
} //namespace epic
